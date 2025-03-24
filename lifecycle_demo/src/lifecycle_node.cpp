#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <std_msgs/msg/string.hpp>

using namespace rclcpp_lifecycle;

class MyLifecycleNode : public LifecycleNode
{
public:
    explicit MyLifecycleNode(const std::string &node_name)
        : LifecycleNode(node_name)
    {
        RCLCPP_INFO(get_logger(), "Lifecycle Node Constructor");
    }

    CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "on_configure() called.");
        publisher_ = this->create_publisher<std_msgs::msg::String>("lifecycle_chatter", 10);
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "on_activate() called.");
        publisher_->on_activate();
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), 
            [this]() {
                auto msg = std_msgs::msg::String();
                msg.data = "Hello from lifecycle node!";
                publisher_->publish(msg);
            });
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "on_deactivate() called.");
        timer_.reset();
        publisher_->on_deactivate();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "on_cleanup() called.");
        publisher_.reset();
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "on_shutdown() called from state: %s", state.label().c_str());
        return CallbackReturn::SUCCESS;
    }

private:
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyLifecycleNode>("my_lifecycle_node");
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
