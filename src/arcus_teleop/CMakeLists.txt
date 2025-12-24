cmake_minimum_required(VERSION 3.8)
project(arcus_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_executable(teleop_key src/teleop_key.cpp)
add_executable(teleop_joy src/teleop_joy.cpp)

target_include_directories(teleop_key PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(teleop_joy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(teleop_key PUBLIC cxx_std_17)
target_compile_features(teleop_joy PUBLIC cxx_std_17)

ament_target_dependencies(teleop_key
  rclcpp
  ackermann_msgs
)
ament_target_dependencies(teleop_joy
  rclcpp
  sensor_msgs
  ackermann_msgs
)

install(TARGETS
  teleop_key
  teleop_joy
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
