cmake_minimum_required(VERSION 3.8)
project(mono_camera_capture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(realsense2 REQUIRED)

add_library(${PROJECT_NAME}_lib
  src/mono_camera_node.cpp
)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  rcl_interfaces
  sensor_msgs
  std_msgs
)

set(REALSENSE_TARGET "")
if(TARGET realsense2::realsense2)
  set(REALSENSE_TARGET realsense2::realsense2)
  target_link_libraries(${PROJECT_NAME}_lib ${REALSENSE_TARGET})
elseif(TARGET realsense2)
  set(REALSENSE_TARGET realsense2)
  target_link_libraries(${PROJECT_NAME}_lib ${REALSENSE_TARGET})
else()
  target_link_libraries(${PROJECT_NAME}_lib ${realsense2_LIBRARY})
endif()

target_include_directories(${PROJECT_NAME}_lib PRIVATE
  ${realsense2_INCLUDE_DIR}
)

add_executable(mono_camera_node
  src/mono_camera_node_main.cpp
)

target_link_libraries(mono_camera_node
  ${PROJECT_NAME}_lib
)

# librealsense 2.58 builds from some non-ROS repositories bundle and export
# Fast DDS symbols.  Loading one of those libraries into a ROS 2 process that
# uses rmw_fastrtps can cause an ABI collision before the node constructor even
# runs.  Preserve the directory of the exact SDK selected by find_package() so
# the executable cannot silently load a same-SONAME library from /usr or
# /usr/local at runtime.
set(REALSENSE_RUNTIME_DIRS "")
if(REALSENSE_TARGET)
  get_target_property(REALSENSE_CONFIGS ${REALSENSE_TARGET} IMPORTED_CONFIGURATIONS)
  foreach(REALSENSE_CONFIG IN LISTS REALSENSE_CONFIGS)
    string(TOUPPER "${REALSENSE_CONFIG}" REALSENSE_CONFIG_UPPER)
    get_target_property(
      REALSENSE_LOCATION ${REALSENSE_TARGET} "IMPORTED_LOCATION_${REALSENSE_CONFIG_UPPER}")
    if(REALSENSE_LOCATION)
      get_filename_component(REALSENSE_RUNTIME_DIR "${REALSENSE_LOCATION}" DIRECTORY)
      list(APPEND REALSENSE_RUNTIME_DIRS "${REALSENSE_RUNTIME_DIR}")
    endif()
  endforeach()
endif()

if(REALSENSE_RUNTIME_DIRS)
  list(REMOVE_DUPLICATES REALSENSE_RUNTIME_DIRS)
  set_target_properties(
    ${PROJECT_NAME}_lib mono_camera_node
    PROPERTIES
      BUILD_RPATH "${REALSENSE_RUNTIME_DIRS}"
      INSTALL_RPATH "${REALSENSE_RUNTIME_DIRS}"
  )
endif()

install(TARGETS
  ${PROJECT_NAME}_lib
  mono_camera_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rcl_interfaces
  sensor_msgs
  std_msgs
)

ament_package()
