cmake_minimum_required(VERSION 3.8)
project(apriltag_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedGoal.msg"
  "msg/DetectedGoalArray.msg"
  "srv/SaveCurrentTagGoal.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

# Executable
add_executable(apriltag_node
  src/Apriltag-node/apriltag_node.cpp
)

# ROS dependencies
ament_target_dependencies(apriltag_node
  rclcpp
  sensor_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  std_msgs
)

target_link_libraries(apriltag_node
  apriltag
  ${OpenCV_LIBRARIES}
)

target_include_directories(apriltag_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

rosidl_target_interfaces(apriltag_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(TARGETS
  apriltag_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/Gaze-tracking/eyetrax_websocket_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
