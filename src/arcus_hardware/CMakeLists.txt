cmake_minimum_required(VERSION 3.8)
project(arcus_hardware)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Include directories
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

# Hardware Interface Plugin (this is what gets loaded by ros2_control)
add_library(${PROJECT_NAME} SHARED
  src/vesc_packet.cpp
  src/vesc_driver.cpp
  src/vesc_interface.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Plugin export
pluginlib_export_plugin_description_file(hardware_interface plugin_description.xml)

# Test executable (standalone, no ROS dependencies needed for basic test)
add_executable(test_vesc
  test/test_vesc.cpp
)

target_include_directories(test_vesc PRIVATE
  include
  ${Boost_INCLUDE_DIRS}
)

target_link_libraries(test_vesc
  ${Boost_LIBRARIES}
  ${PROJECT_NAME} 
  pthread
)

ament_target_dependencies(test_vesc
  rclcpp
)

# Installation
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS test_vesc
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export information
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()