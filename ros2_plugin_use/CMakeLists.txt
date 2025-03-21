cmake_minimum_required(VERSION 3.5)
project(ping_pong_test)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(ping_pong_server src/ping_pong_server.cpp)
ament_target_dependencies(ping_pong_server rclcpp example_interfaces)

add_executable(ping_pong_client src/ping_pong_client.cpp)
ament_target_dependencies(ping_pong_client rclcpp example_interfaces)

install(TARGETS
  ping_pong_server
  ping_pong_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
