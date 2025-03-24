#pragma once
#include "rclcpp/rclcpp.hpp"

namespace LiJun{
  class PluginTest{
    public:
      virtual ~PluginTest(){}
      virtual void DoSomething() = 0;
  };
}
