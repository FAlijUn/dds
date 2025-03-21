cmake_minimum_required(VERSION 3.5)
project(ros2_plugin_use)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_plugin_test REQUIRED)
find_package(pluginlib REQUIRED)

add_executable(ros2_plugin_use src/main.cpp)
target_include_directories(ros2_plugin_use PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ros2_plugin_use rclcpp pluginlib ros2_plugin_test)

install(TARGETS ros2_plugin_use
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
