#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class AgentManagerNode : public rclcpp::Node
{
public:
    AgentManagerNode() : Node("agent_manager")
    {
        RCLCPP_INFO(this->get_logger(), "✅ AgentManagerNode started.");

        status_pub_ = this->create_publisher<std_msgs::msg::String>("/test", 10);

        // 타이머로 1초마다 상태 발행
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&AgentManagerNode::publish_status, this));
    }

private:
    void publish_status()
    {
        std_msgs::msg::String msg;
        msg.data = "idle";  // 예: idle, busy, charging, error ...
        status_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "📤 상태 발행: %s", msg.data.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AgentManagerNode>());
    rclcpp::shutdown();
    return 0;
}
