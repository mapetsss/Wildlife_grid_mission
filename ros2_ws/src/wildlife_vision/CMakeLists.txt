cmake_minimum_required(VERSION 3.8)
project(wildlife_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ros2_yolos_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VisionTrigger.msg"
  "msg/AnimalCount.msg"
  "msg/AnimalReport.msg"
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(wildlife_vision_node
  src/wildlife_vision_node.cpp
)
target_include_directories(wildlife_vision_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${ros2_yolos_cpp_INCLUDE_DIRS}
)
ament_target_dependencies(wildlife_vision_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  ros2_yolos_cpp
)
target_link_libraries(wildlife_vision_node
  ${OpenCV_LIBS}
  ${ros2_yolos_cpp_LIBRARIES}
  "${cpp_typesupport_target}"
)

add_executable(wildlife_vision_test_node
  src/wildlife_vision_test_node.cpp
)
ament_target_dependencies(wildlife_vision_test_node
  rclcpp
)
target_link_libraries(wildlife_vision_test_node
  "${cpp_typesupport_target}"
)

add_executable(wildlife_report_serial_node
  src/wildlife_report_serial_node.cpp
)
ament_target_dependencies(wildlife_report_serial_node
  rclcpp
)
target_link_libraries(wildlife_report_serial_node
  "${cpp_typesupport_target}"
)

install(TARGETS
  wildlife_vision_node
  wildlife_vision_test_node
  wildlife_report_serial_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
