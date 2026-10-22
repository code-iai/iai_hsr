// Copyright 2025 AICOR Institute for Artificial Intelligence
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gmock/gmock.h>
#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

// Simpler test - just verify plugin loads
TEST(TestLoadHsrVelocityController, load_controller)
{
  // Initialize rclcpp with parameter file
  std::string test_file_path = std::string(TEST_PARAMS_DIR) + "/test_controller_params.yaml";
  const char* argv[] = {"test_load_controller", "--ros-args", "--params-file", test_file_path.c_str()};
  int argc = sizeof(argv) / sizeof(char*);
  
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm(
    executor, ros2_control_test_assets::minimal_robot_urdf, true, "test_controller_manager");

  // Controller nodes ignore the global --params-file; pass it per controller
  cm.set_parameter(rclcpp::Parameter("test_hsr_velocity_controller.params_file", test_file_path));

  ASSERT_NE(
    cm.load_controller(
      "test_hsr_velocity_controller", "hsr_velocity_controller_ns/HsrVelocityController"),
    nullptr);

  rclcpp::shutdown();
}
