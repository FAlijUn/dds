#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/set_bool.hpp"

class PingPongServer : public rclcpp::Node
{
public:
  PingPongServer() : Node("ping_pong_server")
  {
    service_ = this->create_service<example_interfaces::srv::SetBool>(
      "ping_pong", 
      std::bind(&PingPongServer::handle_ping, this, std::placeholders::_1, std::placeholders::_2)
    );
    RCLCPP_INFO(this->get_logger(), "Ping-Pong 服务已启动...");
  }

private:
  void handle_ping(const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
                    std::shared_ptr<example_interfaces::srv::SetBool::Response> response)
  {
    if (request->data)
    {
      RCLCPP_INFO(this->get_logger(), "收到 Ping 请求");
      response->success = true;
      response->message = "pong";
    }
    else
    {
      response->success = false;
      response->message = "error";
    }
  }

  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PingPongServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
