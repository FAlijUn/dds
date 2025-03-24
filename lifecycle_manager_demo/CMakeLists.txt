cmake_minimum_required(VERSION 3.8)
project(lifecycle_manager_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

add_executable(lifecycle_manager_demo src/lifecycle_manager.cpp)
ament_target_dependencies(lifecycle_manager_demo rclcpp rclcpp_lifecycle lifecycle_msgs)
install(TARGETS lifecycle_manager_demo DESTINATION lib/${PROJECT_NAME})

ament_package()
