#include <vector>
#include <string>
#include <memory>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <controller_interface/controller_interface.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <urdf/model.h>

namespace hsr_velocity_controller_ns
{

class HsrVelocityController : public controller_interface::ControllerInterface
{
public:
  HsrVelocityController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(joint_names_.size());
    for (const auto &joint_name : joint_names_) {
      config.names.push_back(joint_name + "/position");
    }
    return config;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(joint_names_.size() * 2);
    for (const auto &joint_name : joint_names_) {
      config.names.push_back(joint_name + "/position");
      config.names.push_back(joint_name + "/velocity");
    }
    return config;
  }

  controller_interface::CallbackReturn on_init() override
  {
    auto node = get_node();
    std::string controller_prefix;
    auto param_list = node->list_parameters({}, 10);
    for (const auto &name : param_list.names) {
      if (name.find(".joints") != std::string::npos) {
        controller_prefix = name.substr(0, name.find(".joints"));
        break;
      }
    }
    auto get_param = [&](const std::string &key, rclcpp::Parameter &out) -> bool {
      if (!controller_prefix.empty()) {
        std::string full_key = controller_prefix + "." + key;
        if (node->has_parameter(full_key)) {
          out = node->get_parameter(full_key);
          return true;
        }
      }
      if (node->has_parameter(key)) {
        out = node->get_parameter(key);
        return true;
      }
      return false;
    };

    node->declare_parameter("joints", std::vector<std::string>());
    node->declare_parameter("p_gains", std::vector<double>());
    node->declare_parameter("i_gains", std::vector<double>());
    node->declare_parameter("d_gains", std::vector<double>());
    node->declare_parameter("feedforward_gains", std::vector<double>());

    rclcpp::Parameter joints_param;
    if (!get_param("joints", joints_param)) return controller_interface::CallbackReturn::ERROR;
    joint_names_ = joints_param.as_string_array();
    n_joints_ = joint_names_.size();
    if (n_joints_ == 0) return controller_interface::CallbackReturn::ERROR;

    rclcpp::Parameter p, i, d, ff;
    if (!get_param("p_gains", p) || !get_param("i_gains", i) ||
        !get_param("d_gains", d) || !get_param("feedforward_gains", ff))
      return controller_interface::CallbackReturn::ERROR;
    p_gains_ = p.as_double_array();
    i_gains_ = i.as_double_array();
    d_gains_ = d.as_double_array();
    ff_gains_ = ff.as_double_array();
    if (p_gains_.size() != n_joints_ || i_gains_.size() != n_joints_ ||
        d_gains_.size() != n_joints_ || ff_gains_.size() != n_joints_)
      return controller_interface::CallbackReturn::ERROR;

    commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));
    js_ = std::vector<double>(n_joints_, 0.0);
    old_integrator_ = std::vector<double>(n_joints_, 0.0);
    old_error_ = std::vector<double>(n_joints_, 0.0);
    filtered_vel_ = std::vector<double>(n_joints_, 0.0);

    sub_command_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "command", rclcpp::SystemDefaultsQoS(),
      std::bind(&HsrVelocityController::commandCB, this, std::placeholders::_1));

    auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>("controller_state", 1);
    pub_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(publisher);

    urdf_ = std::make_shared<urdf::Model>();
    if (!node->has_parameter("robot_description")) return controller_interface::CallbackReturn::ERROR;
    std::string urdf_string = node->get_parameter("robot_description").as_string();
    if (!urdf_->initString(urdf_string)) return controller_interface::CallbackReturn::ERROR;

    joints_urdf_.resize(n_joints_);
    for (size_t i = 0; i < n_joints_; i++)
      joints_urdf_[i] = urdf_->getJoint(joint_names_[i]);

    counter = 0;
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override { return controller_interface::CallbackReturn::SUCCESS; }
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    for (size_t i = 0; i < n_joints_; i++) {
      double pos = state_interfaces_[i * 2].get_value();
      js_[i] = pos;
      old_integrator_[i] = 0.0;
      old_error_[i] = 0.0;
      filtered_vel_[i] = 0.0;
      command_interfaces_[i].set_value(js_[i]);
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override { return controller_interface::CallbackReturn::SUCCESS; }

  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &period) override
  {
    auto &commands = *commands_buffer_.readFromRT();
    double dt = period.seconds();

    for (size_t i = 0; i < n_joints_; i++) {
      double vel_cmd = commands[i];
      double current_pos = state_interfaces_[i * 2].get_value();
      double current_vel = state_interfaces_[i * 2 + 1].get_value();

      filtered_vel_[i] = 0.95 * current_vel + 0.05 * filtered_vel_[i];

      if (vel_cmd == 0.0) {
        js_[i] = current_pos;
        old_integrator_[i] = 0.0;
        old_error_[i] = 0.0;
      } else {
        double error = vel_cmd - filtered_vel_[i];
        double new_integrator = old_integrator_[i] + error * dt;
        double next_pos = current_pos + vel_cmd * dt * ff_gains_[i]
                          + error * p_gains_[i]
                          + new_integrator * i_gains_[i]
                          + d_gains_[i] / dt * (error - old_error_[i]);

        if (joints_urdf_[i] && joints_urdf_[i]->limits)
          next_pos = std::min(std::max(next_pos, joints_urdf_[i]->limits->lower),
                              joints_urdf_[i]->limits->upper);

        old_integrator_[i] = new_integrator;
        command_interfaces_[i].set_value(next_pos);
        old_error_[i] = error;
      }
    }

    if (counter % 10 == 0 && pub_ && pub_->trylock()) {
      pub_->msg_.data = filtered_vel_;
      pub_->unlockAndPublish();
    }
    counter++;
    return controller_interface::return_type::OK;
  }

private:
  std::vector<std::string> joint_names_;
  unsigned int n_joints_;
  unsigned int counter{0};

  std::vector<double> js_;
  std::vector<double> old_integrator_;
  std::vector<double> old_error_;
  std::vector<double> filtered_vel_;
  std::vector<double> p_gains_;
  std::vector<double> i_gains_;
  std::vector<double> d_gains_;
  std::vector<double> ff_gains_;
  std::vector<urdf::JointConstSharedPtr> joints_urdf_;
  std::shared_ptr<urdf::Model> urdf_;

  realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_command_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> pub_;

  void commandCB(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != n_joints_) return;
    commands_buffer_.writeFromNonRT(msg->data);
  }
};

} 

PLUGINLIB_EXPORT_CLASS(hsr_velocity_controller_ns::HsrVelocityController, controller_interface::ControllerInterface)
