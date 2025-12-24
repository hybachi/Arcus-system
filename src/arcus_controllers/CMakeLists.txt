cmake_minimum_required(VERSION 3.8)
project(arcus_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED src/ackermann_controller.cpp)

target_include_directories(
  ${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(
  ${PROJECT_NAME} 
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ackermann_msgs
)

pluginlib_export_plugin_description_file(controller_interface plugin_description.xml)

install(TARGETS ${PROJECT_NAME} 
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)

install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME})

install(FILES plugin_description.xml
        DESTINATION share/${PROJECT_NAME})

ament_export_libraries(${PROJECT_NAME})
ament_package()
