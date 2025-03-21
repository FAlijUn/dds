#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/set_bool.hpp"

class PingPongClient : public rclcpp::Node
{
public:
  PingPongClient() : Node("ping_pong_client")
  {
    client_ = this->create_client<example_interfaces::srv::SetBool>("ping_pong");

    // 等待服务端启动
    while (!client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_WARN(this->get_logger(), "等待 Ping-Pong 服务...");
    }

    // 每秒发送 Ping 请求
    timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                      std::bind(&PingPongClient::send_ping, this));
  }

private:
  void send_ping()
  {
    auto request = std::make_shared<example_interfaces::srv::SetBool::Request>();
    request->data = true;

    auto future = client_->async_send_request(request,
        std::bind(&PingPongClient::handle_response, this, std::placeholders::_1));
  }

  void handle_response(rclcpp::Client<example_interfaces::srv::SetBool>::SharedFuture future)
  {
    try
    {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "收到响应: %s", response->message.c_str());
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "获取响应失败: %s", e.what());
    }
  }
  rclcpp::Client<example_interfaces::srv::SetBool>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PingPongClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
