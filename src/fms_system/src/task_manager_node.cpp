#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TaskManagerNode : public rclcpp::Node
{
public:
    TaskManagerNode() : Node("task_manager")
    {
        RCLCPP_INFO(this->get_logger(), "TaskManagerNode started.");

        // 작업 명령 구독
        task_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/task_cmd", 10,
            std::bind(&TaskManagerNode::task_callback, this, std::placeholders::_1));

        // 작업 상태 퍼블리시
        task_pub_ = this->create_publisher<std_msgs::msg::String>("/task_status", 10);
    }

private:
    void task_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "작업 수신됨: %s", msg->data.c_str());

        auto status = std_msgs::msg::String();
        status.data = "작업 수행중: " + msg->data;
        task_pub_->publish(status);
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TaskManagerNode>());
    rclcpp::shutdown();
    return 0;
}
