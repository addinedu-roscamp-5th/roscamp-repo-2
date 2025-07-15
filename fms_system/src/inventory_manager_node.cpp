#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class InventoryManagerNode : public rclcpp::Node
{
public:
    InventoryManagerNode() : Node("inventory_manager")
    {
        RCLCPP_INFO(this->get_logger(), "InventoryManagerNode started.");
        // TODO: Add inventory database access and service
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InventoryManagerNode>());
    rclcpp::shutdown();
    return 0;
}