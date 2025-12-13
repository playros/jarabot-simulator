cmake_minimum_required(VERSION 3.8)
project(jarabot_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(jarabot_sim_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(jara_sim_encoder src/jara_sim_encoder.cpp)
ament_target_dependencies(jara_sim_encoder
  rclcpp
  geometry_msgs
  jarabot_sim_interfaces
)

add_executable(jara_sim_odometry src/jara_sim_odometry.cpp)
ament_target_dependencies(jara_sim_odometry
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  jarabot_sim_interfaces
)

add_executable(jara_sim_lidar src/jara_sim_lidar.cpp)
ament_target_dependencies(jara_sim_lidar
  rclcpp
  nav_msgs
  sensor_msgs
)

add_executable(jara_sim_marker src/jara_sim_marker.cpp)
ament_target_dependencies(jara_sim_marker
  rclcpp
  visualization_msgs
)

add_executable(jara_sim_path src/jara_sim_path.cpp)
ament_target_dependencies(jara_sim_path
  rclcpp
  nav_msgs
  geometry_msgs
)

install(TARGETS
  jara_sim_encoder
  jara_sim_odometry
  jara_sim_lidar
  jara_sim_marker
  jara_sim_path
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
