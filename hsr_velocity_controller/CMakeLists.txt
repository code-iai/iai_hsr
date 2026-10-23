cmake_minimum_required(VERSION 3.5)
project(hsr_velocity_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(urdf REQUIRED)
add_library(${PROJECT_NAME} SHARED src/hsr_velocity_controller.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME} rclcpp controller_interface hardware_interface pluginlib std_msgs realtime_tools urdf)

# Install directories
install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
)

# Install plugin XML
install(FILES hsr_velocity_controller.xml
        DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME})

# Export plugin description
pluginlib_export_plugin_description_file(controller_interface hsr_velocity_controller.xml)

# Export package
ament_export_dependencies(rclcpp controller_interface hardware_interface pluginlib std_msgs realtime_tools)


if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_controller test/test_load_controller.cpp)
  target_include_directories(test_load_controller PRIVATE include)
  target_compile_definitions(test_load_controller PRIVATE TEST_PARAMS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test")
  ament_target_dependencies(test_load_controller
    rclcpp
    controller_manager
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )
endif()

# Verification executable to check plugin registration
add_executable(verify_plugin_registration test/verify_plugin_registration.cpp)
target_include_directories(verify_plugin_registration PRIVATE include)
ament_target_dependencies(verify_plugin_registration
  controller_interface
  pluginlib
)
install(TARGETS verify_plugin_registration
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
