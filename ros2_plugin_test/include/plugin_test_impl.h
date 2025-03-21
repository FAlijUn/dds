#pragma once
#include "plugin_test_interface.h"

namespace LiJun{
  class PluginTestImpl:public PluginTest{
    public:
      void DoSomething() override;
  };
}
