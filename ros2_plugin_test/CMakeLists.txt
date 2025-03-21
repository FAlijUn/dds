cmake_minimum_required(VERSION 3.5)
project(ros2_plugin_test)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

add_library(plugin_test_impl src/plugin_test_impl.cpp)
target_include_directories(plugin_test_impl PUBLIC include)
ament_target_dependencies(plugin_test_impl rclcpp pluginlib)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS plugin_test_impl
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
