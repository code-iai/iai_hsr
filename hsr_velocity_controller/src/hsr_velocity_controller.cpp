#include <vector>
#include <string>
#include <memory>

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
    for (const auto& joint_name : joint_names_) {
      config.names.push_back(joint_name + "/position");
    }
    return config;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.reserve(joint_names_.size() * 2);
    for (const auto& joint_name : joint_names_) {
      config.names.push_back(joint_name + "/position");
      config.names.push_back(joint_name + "/velocity");
    }
    return config;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override
  {
    try {
      auto node = get_node();

      // Declare parameters first
      node->declare_parameter("joints", std::vector<std::string>());
      node->declare_parameter("p_gains", std::vector<double>());
      node->declare_parameter("i_gains", std::vector<double>());
      node->declare_parameter("d_gains", std::vector<double>());
      node->declare_parameter("feedforward_gains", std::vector<double>());
      node->declare_parameter("robot_description", std::string(""));

      // Get joint names parameter
      joint_names_ = node->get_parameter("joints").as_string_array();

      n_joints_ = joint_names_.size();

      if (n_joints_ == 0) {
        RCLCPP_ERROR(get_node()->get_logger(), "List of joint names is empty.");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
      }

      RCLCPP_INFO(get_node()->get_logger(), "Controller configured for %zu joints:", n_joints_);
      for (size_t i = 0; i < n_joints_; i++) {
        RCLCPP_INFO(get_node()->get_logger(), "  - %s", joint_names_[i].c_str());
      }

      commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));

      // Create subscriber
      sub_command_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
        "command",
        rclcpp::SystemDefaultsQoS(),
        std::bind(&HsrVelocityController::commandCB, this, std::placeholders::_1)
      );

      // Create publisher first, then create realtime publisher
      auto publisher = node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "controller_state", 1);
      pub_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(publisher);

      // Load URDF
      urdf_ = std::make_shared<urdf::Model>();
      std::string urdf_string = node->get_parameter("robot_description").as_string();
      if (!urdf_->initString(urdf_string)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse urdf file");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
      }

      counter = 0;
      js_ = std::vector<double>(n_joints_);
      joints_urdf_ = std::vector<urdf::JointConstSharedPtr>(n_joints_);

      for (unsigned int i = 0; i < n_joints_; i++) {
        js_[i] = 0.0;
        joints_urdf_[i] = urdf_->getJoint(joint_names_[i]);
        if (joints_urdf_[i] && joints_urdf_[i]->limits) {
          RCLCPP_INFO_STREAM(get_node()->get_logger(), "Joint " << joint_names_[i] << " limits: ["
                            << joints_urdf_[i]->limits->lower << ", " << joints_urdf_[i]->limits->upper << "]");
        } else {
          RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Joint " << joint_names_[i] << " has no limits defined!");
          return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
        }
      }

      // Get PID gains with default values
      p_gains_ = node->get_parameter("p_gains").as_double_array();
      i_gains_ = node->get_parameter("i_gains").as_double_array();
      d_gains_ = node->get_parameter("d_gains").as_double_array();
      ff_gains_ = node->get_parameter("feedforward_gains").as_double_array();

      // Check gain sizes
      if (p_gains_.size() != n_joints_ || i_gains_.size() != n_joints_ ||
          d_gains_.size() != n_joints_ || ff_gains_.size() != n_joints_) {
        RCLCPP_ERROR(get_node()->get_logger(), "Gain vector sizes do not match number of joints!");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
      }

      RCLCPP_INFO(get_node()->get_logger(), "Gains loaded successfully:");
      for (size_t i = 0; i < n_joints_; i++) {
        RCLCPP_INFO(get_node()->get_logger(), "Joint %zu: P=%.3f, I=%.3f, D=%.3f, FF=%.3f",
                   i, p_gains_[i], i_gains_[i], d_gains_[i], ff_gains_[i]);
      }

      old_integrator_ = std::vector<double>(n_joints_, 0.0);
      old_error_ = std::vector<double>(n_joints_, 0.0);
      filtered_vel_ = std::vector<double>(n_joints_, 0.0);

    } catch (const std::exception& e) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Exception thrown in init: " << e.what());
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override
  {
    std::vector<double>& commands = *commands_buffer_.readFromRT();

    double dt = period.seconds();

    // Debug: print first command occasionally
    if (counter % 100 == 0 && commands[0] != 0.0) {
      RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                          "Received command: %.3f", commands[0]);
    }

    for (unsigned int i = 0; i < n_joints_; i++) {
      double vel_cmd = commands[i];

      // Get current state
      double current_pos = state_interfaces_[i * 2].get_value();
      double current_vel = state_interfaces_[i * 2 + 1].get_value();

      // Debug occasionally
      if (counter % 100 == 0 && i == 0 && vel_cmd != 0.0) {
        RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                            "Joint %d: cmd=%.3f, pos=%.3f, vel=%.3f",
                            i, vel_cmd, current_pos, current_vel);
      }

      // Filter the velocity
      float alpha = 0.95f;
      filtered_vel_[i] = (1.0f - alpha) * filtered_vel_[i] + alpha * current_vel;

      if (vel_cmd == 0.0) {
        js_[i] = current_pos;
        old_integrator_[i] = 0.0;
        old_error_[i] = 0.0;
      } else {
        double error = (vel_cmd - filtered_vel_[i]);
        double new_integrator = old_integrator_[i] + error * dt;

        double next_pos = current_pos + vel_cmd * dt * ff_gains_[i]
                          + error * p_gains_[i]
                          + new_integrator * i_gains_[i]
                          + d_gains_[i] / dt * (error - old_error_[i]);

        // Clamp the output by the joint limits
        if (joints_urdf_[i] && joints_urdf_[i]->limits) {
          if (next_pos > joints_urdf_[i]->limits->upper) {
            next_pos = joints_urdf_[i]->limits->upper;
            if (error < 0) {
              old_integrator_[i] = new_integrator;
            }
          } else if (next_pos < joints_urdf_[i]->limits->lower) {
            next_pos = joints_urdf_[i]->limits->lower;
            if (error > 0) {
              old_integrator_[i] = new_integrator;
            }
          } else {
            old_integrator_[i] = new_integrator;
          }
        } else {
          old_integrator_[i] = new_integrator;
        }

        bool success = command_interfaces_[i].set_value(next_pos);
        if (!success) {
          RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                               "Failed to set command for joint %d", i);
        }
        old_error_[i] = error;

        // Debug output position
        if (counter % 100 == 0 && i == 0) {
          RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                              "Setting joint %d position to: %.3f", i, next_pos);
        }
      }
    }

    // Publish controller state periodically
    if (counter % 10 == 0 && pub_ && pub_->trylock()) {
      pub_->msg_.data = filtered_vel_;
      pub_->unlockAndPublish();
    }
    counter++;

    return controller_interface::return_type::OK;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override
  {
    RCLCPP_INFO(get_node()->get_logger(), "Configuring controller");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override
  {
    RCLCPP_INFO(get_node()->get_logger(), "Activating controller");

    // Reset controller state
    for (unsigned int i = 0; i < n_joints_; i++) {
      double current_pos = state_interfaces_[i * 2].get_value();
      js_[i] = current_pos;
      old_integrator_[i] = 0.0;
      old_error_[i] = 0.0;
      filtered_vel_[i] = 0.0;

      RCLCPP_INFO(get_node()->get_logger(), "Joint %d initial position: %.3f", i, current_pos);
    }

    // Set initial command to current position
    for (unsigned int i = 0; i < n_joints_; i++) {
      bool success = command_interfaces_[i].set_value(js_[i]);
      if (!success) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to set initial command for joint %d", i);
      } else {
        RCLCPP_INFO(get_node()->get_logger(), "Set joint %d initial command to: %.3f", i, js_[i]);
      }
    }

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override
  {
    RCLCPP_INFO(get_node()->get_logger(), "Deactivating controller");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private:
  std::vector<std::string> joint_names_;
  realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer_;
  unsigned int n_joints_;
  unsigned int counter;
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

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_command_;
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>> pub_;

  void commandCB(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != n_joints_) {
      RCLCPP_ERROR_STREAM(get_node()->get_logger(), "Dimension of command (" << msg->data.size()
                        << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
      return;
    }
    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                        "Received command with %zu elements", msg->data.size());
    commands_buffer_.writeFromNonRT(msg->data);
  }
};

}  // namespace hsr_velocity_controller_ns

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  hsr_velocity_controller_ns::HsrVelocityController,
  controller_interface::ControllerInterface)