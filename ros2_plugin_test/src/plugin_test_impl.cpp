#include"plugin_test_impl.h"

namespace LiJun{
  void PluginTestImpl::DoSomething(){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DoSomething() is called");
  }
}

#include "pluginlib/class_list_macros.hpp"
  PLUGINLIB_EXPORT_CLASS(LiJun::PluginTestImpl, LiJun::PluginTest)