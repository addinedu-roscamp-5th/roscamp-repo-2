#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MonitoringManagerNode : public rclcpp::Node
{
public:
    MonitoringManagerNode() : Node("monitoring_manager")
    {
        RCLCPP_INFO(this->get_logger(), "✅ MonitoringManagerNode started.");

        // 에이전트 위치 구독
        battery_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/pinky_battery_present", 10,
            std::bind(&MonitoringManagerNode::battery_callback, this, std::placeholders::_1));

        // 에이전트 상태 구독
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/tracked_pose", 10,
            std::bind(&MonitoringManagerNode::pose_callback, this, std::placeholders::_1));
        
        // cmd_vel 구독
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MonitoringManagerNode::cmd_vel_callback, this, std::placeholders::_1));
    }

private:
    void battery_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "📍 Received data: %.2f", msg->data);
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "📡 Tracked Pose: \nx = %.2f \ny = %.2f \nz = %.2f \n",
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z);

    }


    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "🚗 cmd_vel: linear.x=%.2f, angular.z=%.2f", msg->linear.x, msg->angular.z / 5);
    }

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MonitoringManagerNode>());
    rclcpp::shutdown();
    return 0;
}