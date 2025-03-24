cmake_minimum_required(VERSION 3.5)
project(ros2_plugin_test)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

add_library(plugin_test_impl SHARED src/plugin_test_impl.cpp)
target_include_directories(plugin_test_impl PUBLIC   
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(plugin_test_impl rclcpp pluginlib)

pluginlib_export_plugin_description_file(ros2_plugin_test plugins.xml) # 包名

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS plugin_test_impl
  EXPORT plugin_test_impl_export  
  ARCHIVE DESTINATION lib        
  LIBRARY DESTINATION lib     
  RUNTIME DESTINATION bin        
)

ament_export_targets(plugin_test_impl_export)
ament_export_include_directories(include)
ament_export_dependencies(pluginlib rclcpp)
ament_package()
