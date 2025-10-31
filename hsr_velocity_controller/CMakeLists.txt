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
add_library(${PROJECT_NAME} SHARED src/hsr_velocity_controller.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME} rclcpp controller_interface hardware_interface pluginlib std_msgs realtime_tools)

# Install directories
install(DIRECTORY launch config src
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

# Export plugin description
pluginlib_export_plugin_description_file(controller_interface hsr_velocity_controller.xml)

# Export package
ament_export_dependencies(rclcpp controller_interface hardware_interface pluginlib std_msgs realtime_tools)

ament_package()
