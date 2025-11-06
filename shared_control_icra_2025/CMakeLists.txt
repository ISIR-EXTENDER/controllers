cmake_minimum_required(VERSION 3.8)
project(shared_control_icra_2025)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/cartesian_velocity_teleop_controller.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_components
  geometry_msgs
  robot_interfaces
  Eigen3
)

# Register your controller class as a component
rclcpp_components_register_nodes(${PROJECT_NAME}
  "cartesian_velocity_controller::CartesianVelocityTeleopController"
)

# Install the library
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the entire config directory (containing controllers.yaml + plugin XML)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Export using pluginlib so the controller_interface can find the custom controller
pluginlib_export_plugin_description_file(
        controller_interface config/cartesian_velocity_controller.xml)

ament_package()