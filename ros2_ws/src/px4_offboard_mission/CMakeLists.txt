cmake_minimum_required(VERSION 3.8)
project(px4_offboard_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(wildlife_vision REQUIRED)

add_executable(offboard_mission_node
  src/offboard_mission_node.cpp
)

target_compile_features(offboard_mission_node PUBLIC cxx_std_17)

ament_target_dependencies(offboard_mission_node
  rclcpp
  geometry_msgs
  mavros_msgs
)

add_executable(wildlife_mission_node
  src/wildlife_mission_node.cpp
)

target_compile_features(wildlife_mission_node PUBLIC cxx_std_17)

ament_target_dependencies(wildlife_mission_node
  rclcpp
  geometry_msgs
  mavros_msgs
  sensor_msgs
  std_msgs
  std_srvs
  wildlife_vision
)

add_executable(small_grid_test_node
  src/small_grid_test_node.cpp
)

target_compile_features(small_grid_test_node PUBLIC cxx_std_17)

ament_target_dependencies(small_grid_test_node
  rclcpp
  geometry_msgs
  mavros_msgs
  nav_msgs
  std_msgs
  std_srvs
)

add_executable(serial_route_bridge_node
  src/serial_route_bridge_node.cpp
)

target_compile_features(serial_route_bridge_node PUBLIC cxx_std_17)

ament_target_dependencies(serial_route_bridge_node
  rclcpp
  std_msgs
  std_srvs
  wildlife_vision
)

install(TARGETS
  offboard_mission_node
  wildlife_mission_node
  small_grid_test_node
  serial_route_bridge_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
