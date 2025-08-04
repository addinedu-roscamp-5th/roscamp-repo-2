#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unordered_map>
#include <string>
#include <memory>
#include <mutex>
#include <vector>
#include <sstream>  // 상단에 필요

using std::placeholders::_1;

enum class RobotType {
  MOBILE,
  ARM,
};

struct RobotInfo {
  std::string robot_id;
  RobotType type;
};

// ───────────────────────
// 상태 저장 구조
// ───────────────────────
struct MobileStatus {
  float battery = -1.0f;
  geometry_msgs::msg::Pose pose;
  float velocity = 0.0f;
};

struct ArmStatus {
  float battery = -1.0f;
  geometry_msgs::msg::Pose pose;
  std::vector<double> joint_angles;
};

// ───────────────────────
// 상태 매니저 클래스
// ───────────────────────
class RobotStateManager : public rclcpp::Node {
public:
  RobotStateManager(const std::vector<RobotInfo>& robots)
    : Node("robot_state_manager")
  {
    for (const auto& robot : robots)
      register_robot(robot);
  }

private:
  std::mutex mutex_;

  std::unordered_map<std::string, RobotType> robot_types_;
  std::unordered_map<std::string, MobileStatus> mobile_states_;
  std::unordered_map<std::string, ArmStatus> arm_states_;

  // 토픽별 Subscription 저장
  std::unordered_map<std::string, rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> battery_subs_;
  std::unordered_map<std::string, rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr> pose_subs_;
  std::unordered_map<std::string, rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr> velocity_subs_;
  std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> joint_state_subs_;

  void register_robot(const RobotInfo& robot)
  {
    const std::string& id = robot.robot_id;
    robot_types_[id] = robot.type;

    // 공통 토픽: battery
    std::string battery_topic = "/" + id + "/battery";
    battery_subs_[id] = this->create_subscription<std_msgs::msg::Float32>(
      battery_topic, 10,
      [this, id](const std_msgs::msg::Float32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        float battery_value = msg->data;
        
        if (robot_types_[id] == RobotType::MOBILE) {
          mobile_states_[id].battery = battery_value;
          RCLCPP_INFO(this->get_logger(), "[%s] MOBILE battery: %.2f%%", id.c_str(), battery_value);
        } else {
          arm_states_[id].battery = battery_value;
          RCLCPP_INFO(this->get_logger(), "[%s] ARM battery: %.2f%%", id.c_str(), battery_value);
        }
      }
    );

    // 공통 토픽: pose
    std::string pose_topic = "/" + id + "/camera_pose";
    pose_subs_[id] = this->create_subscription<geometry_msgs::msg::Pose>(
      pose_topic, 10,
      [this, id](const geometry_msgs::msg::Pose::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (robot_types_[id] == RobotType::MOBILE)
          mobile_states_[id].pose = *msg;
        else
          arm_states_[id].pose = *msg;
      }
    );

    // 타입별 토픽
    if (robot.type == RobotType::MOBILE) {
      std::string velocity_topic = "/" + id + "/cmd_vel";
      velocity_subs_[id] = this->create_subscription<std_msgs::msg::Float32>(
        velocity_topic, 10,
        [this, id](const std_msgs::msg::Float32::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          mobile_states_[id].velocity = msg->data;
        }
      );
    } else if (robot.type == RobotType::ARM) {
      std::string joint_topic = "/" + id + "/joint_states";
      joint_state_subs_[id] = this->create_subscription<sensor_msgs::msg::JointState>(
        joint_topic, 10,
        [this, id](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          arm_states_[id].joint_angles = msg->position;

          // joint_angles -> 문자열로 변환
          std::ostringstream oss;
          for (size_t i = 0; i < msg->position.size(); ++i) {
            oss << msg->position[i];
            if (i < msg->position.size() - 1) oss << ", ";
          }

          RCLCPP_INFO(this->get_logger(), "[%s] joint_angles: [%s]",
                      id.c_str(), oss.str().c_str());

          // RCLCPP_INFO(this->get_logger(), "[%s] joint_angles:", id.c_str());
          // for (size_t i = 0; i < msg->position.size(); ++i) {
          //   RCLCPP_INFO(this->get_logger(), "  joint[%zu]: %.4f", i, msg->position[i]);
          // }

        }
      );
    }

    RCLCPP_INFO(this->get_logger(), "Registered robot: %s (%s)", id.c_str(),
                robot.type == RobotType::MOBILE ? "MOBILE" : "ARM");
  }
};
#include <sstream>  // 로그 출력용
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  std::vector<RobotInfo> robots = {
    { "robot1", RobotType::MOBILE },
    { "robot2", RobotType::MOBILE },
    { "robot3", RobotType::MOBILE },
    { "robot4", RobotType::ARM },
    { "robot5", RobotType::ARM }
  };

  auto node = std::make_shared<RobotStateManager>(robots);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
