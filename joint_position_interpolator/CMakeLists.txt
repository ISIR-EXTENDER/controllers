cmake_minimum_required(VERSION 3.8)
project(joint_position_interpolator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdf REQUIRED)
find_package(rosidl_default_generators REQUIRED)


# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointPositionCommand.msg"
  DEPENDENCIES std_msgs
)

add_library(${PROJECT_NAME}_lib SHARED
  src/joint_position_interpolator.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  include
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rclcpp_components
  robot_interfaces
  std_msgs
  sensor_msgs
  Eigen3
  urdf
)

rosidl_target_interfaces(${PROJECT_NAME}_lib
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


# Register your controller class as a component
rclcpp_components_register_nodes(${PROJECT_NAME}_lib
  "joint_controllers::JointPositionInterpolator"
)

# Install the library
install(TARGETS
  ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the entire config directory (containing controllers.yaml + plugin XML)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Install the launch directory
install(DIRECTORY
  config launch 
  DESTINATION share/${PROJECT_NAME}
)

# Export using pluginlib so the controller_interface can find the custom controller
pluginlib_export_plugin_description_file(
        controller_interface config/joint_position_interpolator.xml)

ament_export_dependencies(rosidl_default_runtime)

ament_export_libraries(${PROJECT_NAME}_lib)

ament_package()
