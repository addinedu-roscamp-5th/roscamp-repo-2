import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

rclpy.init()
goal_node=rclpy.create_node('goal_node')
def publish_goal():
    goal_pose.header.stamp = goal_node.get_clock().now().to_msg()
    goal_publisher.publish(goal_pose)
    print("📤 Goal pose published!")
    # print(REAL_WIDTH, REAL_HEIGHT)


goal_pose = PoseStamped()
goal_pose.header.frame_id = 'map'
goal_pose.pose.position.x = 1.0
goal_pose.pose.position.y = 0.13
goal_pose.pose.orientation.w = 0.707
goal_pose.pose.orientation.z = 0.707

pose_sub = goal_node.create_subscription(PoseStamped, '/goal_pose', lambda msg: callback_goal_pose(msg), 10)
def callback_goal_pose(msg):
    print(f"x: {msg.pose.position.x}, y: {msg.pose.position.y}")
# 타이머 등록 (0.5초마다 퍼블리시)
# timer = goal_node.create_timer(0.5, publish_goal)

goal_publisher = goal_node.create_publisher(PoseStamped, '/goal_pose', 10)
goal_publisher.publish(goal_pose)
rclpy.spin_once(goal_node, timeout_sec=1.0)
