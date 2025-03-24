#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>
#include "ros2_plugin_test/plugin_test_interface.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("plugin_test_node");

  pluginlib::ClassLoader<LiJun::PluginTest> loader("ros2_plugin_test", "LiJun::PluginTest");

  try {
    std::shared_ptr<LiJun::PluginTest> plugin = loader.createSharedInstance("LiJun::PluginTestImpl");
    plugin->DoSomething();
  } catch (const pluginlib::LibraryLoadException& e) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load plugin: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}
