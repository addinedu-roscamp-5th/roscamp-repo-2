import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

def main():
    rclpy.init()
    node = rclpy.create_node('single_goal_publisher')

    goal_publisher = node.create_publisher(PoseStamped, '/goal_pose', 10)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = node.get_clock().now().to_msg()
    goal_pose.pose.position.x = 1.0
    goal_pose.pose.position.y = 0.13
    goal_pose.pose.orientation.z = 0.707
    goal_pose.pose.orientation.w = 0.707

    # 퍼블리시 한번만 하고 종료
    goal_publisher.publish(goal_pose)
    print(goal_pose.header.stamp)
    node.get_logger().info('📤 Goal pose published!')
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
