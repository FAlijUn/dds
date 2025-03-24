#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>

using namespace std::chrono_literals;

class LifecycleManager : public rclcpp::Node {
public:
    LifecycleManager() : Node("lifecycle_manager") {
        RCLCPP_INFO(this->get_logger(), "Lifecycle Manager Node Started");
        client_ = this->create_client<lifecycle_msgs::srv::ChangeState>("my_lifecycle_node/change_state");

        // 等待服务可用
        while (!client_->wait_for_service(3s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
                return;
            }
            RCLCPP_WARN(this->get_logger(), "Service not available, waiting...");
        }
        RCLCPP_INFO(this->get_logger(), "Service connected!");

        manage_lifecycle();
    }

    void manage_lifecycle() {
        // 配置
        if (call_service(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
            RCLCPP_INFO(this->get_logger(), "Node Configured");
        }

        // 激活
        if (call_service(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)) {
            RCLCPP_INFO(this->get_logger(), "Node Activated");
        }

        std::this_thread::sleep_for(5s);

        // 停用
        if (call_service(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
            RCLCPP_INFO(this->get_logger(), "Node Deactivated");
        }

        // 清理
        if (call_service(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP)) {
            RCLCPP_INFO(this->get_logger(), "Node Cleaned Up");
        }
    }

private:
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client_;

    bool call_service(uint8_t transition_id) {
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition_id;

        auto future = client_->async_send_request(request);
        if (future.wait_for(5s) == std::future_status::ready) {
            auto response = future.get();
            if (response->success) {
                RCLCPP_INFO(this->get_logger(), "Transition %d succeeded", transition_id);
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Transition %d failed", transition_id);
                return false;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call timeout for transition %d", transition_id);
            return false;
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto manager = std::make_shared<LifecycleManager>();
    rclcpp::spin(manager);
    rclcpp::shutdown();
    return 0;
}