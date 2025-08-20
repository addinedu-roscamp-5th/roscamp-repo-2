// bolt_fms_core.cpp - 리눅스 기반 TCP 서버
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unordered_map>
#include <string>
#include <memory>
#include <mutex>
#include <vector>
#include <thread>
#include <sstream>
#include <chrono>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include <signal.h>

using json = nlohmann::json;

constexpr int TCP_PORT = 9999;

enum class RobotType
{
  MOBILE,
  ARM
};

struct RobotInfo
{
  std::string robot_id;
  RobotType type;
};

// ✅ PoseStamped로 수정
struct MobileStatus
{
  float battery = -1.0f;
  geometry_msgs::msg::PoseStamped pose;
  float velocity = 0.0f;
};

struct ArmStatus
{
  float battery = -1.0f;
  geometry_msgs::msg::PoseStamped pose;
  std::vector<double> joint_angles;
};

class RobotStateManager : public rclcpp::Node
{
public:
  RobotStateManager(const std::vector<RobotInfo>& robots) : Node("robot_state_manager"), tcp_running_(true)
  {
    for (const auto& robot : robots)
      register_robot(robot);
    tcp_thread_ = std::thread([this]() { this->run_tcp_server(); });
  }

  ~RobotStateManager()
  {
    tcp_running_ = false;
    if (tcp_thread_.joinable())
      tcp_thread_.join();
  }

private:
  std::mutex mutex_;
  std::unordered_map<std::string, RobotType> robot_types_;
  std::unordered_map<std::string, MobileStatus> mobile_states_;
  std::unordered_map<std::string, ArmStatus> arm_states_;
  std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> subs_;
  std::thread tcp_thread_;
  bool tcp_running_ = true;

  template <typename MsgT>
  void add_sub(const std::string& topic_suffix, const std::string& id,
               std::function<void(typename MsgT::SharedPtr)> callback)
  {
    auto topic = "/" + id + topic_suffix;
    auto sub = this->create_subscription<MsgT>(topic, 10, callback);
    subs_[id + topic_suffix] = sub;
  }

  void register_robot(const RobotInfo& robot)
  {
    const std::string& id = robot.robot_id;
    robot_types_[id] = robot.type;

    // 🔋 배터리
    add_sub<std_msgs::msg::Float32>("/battery", id, [this, id](auto msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      float val = msg->data;
      if (robot_types_[id] == RobotType::MOBILE)
        mobile_states_[id].battery = val;
      else
        arm_states_[id].battery = val;
      // RCLCPP_INFO(this->get_logger(), "✅ battery : %f",
      //               val);
    });

    // 📷 PoseStamped
    add_sub<geometry_msgs::msg::PoseStamped>("/camera_pose", id, [this, id](auto msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (robot_types_[id] == RobotType::MOBILE) {
        mobile_states_[id].pose = *msg;
      } else {
        arm_states_[id].pose = *msg;
      }
    });

    if (robot.type == RobotType::MOBILE)
    {
      add_sub<std_msgs::msg::Float32>("/cmd_vel", id, [this, id](auto msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        mobile_states_[id].velocity = msg->data;
      });
    }
    else
    {
      add_sub<sensor_msgs::msg::JointState>("/joint_states", id, [this, id](auto msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        arm_states_[id].joint_angles = msg->position;
      });
    }
  }

  std::string get_robot_status_json()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    json j;

    for (const auto& [id, type] : robot_types_)
    {
      geometry_msgs::msg::PoseStamped pose;

      double roll = 0.0, pitch = 0.0, yaw = 0.0;
      if (type == RobotType::MOBILE)
      {
        auto& s = mobile_states_[id];
        pose = s.pose;

        tf2::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y,
                          pose.pose.orientation.z, pose.pose.orientation.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        j[id] = { { "type", "MOBILE" },
                  { "task", "IDLE" },
                  { "status", "PENDING" },
                  { "battery", s.battery },
                  { "pose", { { "x", pose.pose.position.x },
                              { "y", pose.pose.position.y },
                              { "yaw", yaw } } },
                  { "velocity", s.velocity },
                  { "joints", json::array() } };
      }
      else
      {  // ARM
        auto& s = arm_states_[id];
        pose = s.pose;

        tf2::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y,
                          pose.pose.orientation.z, pose.pose.orientation.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        j[id] = { { "type", "ARM" },
                  { "task", "UNLOAD" },
                  { "status", "INPROGRESS(PICK)" },
                  { "battery", s.battery },
                  { "pose", { { "x", pose.pose.position.x },
                              { "y", pose.pose.position.y },
                              { "yaw", yaw } } },
                  { "velocity", 0.0 },
                  { "joints", s.joint_angles } };
      }
    }

    return j.dump();
  }

  void run_tcp_server()
  {
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "소켓 생성 실패");
      return;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(TCP_PORT);

    if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "바인드 실패");
      close(server_fd);
      return;
    }

    if (listen(server_fd, 5) < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "리스닝 실패");
      close(server_fd);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "📡 TCP 서버 대기 중 (포트 %d)...", TCP_PORT);

    while (tcp_running_)
    {
      sockaddr_in client_addr;
      socklen_t len = sizeof(client_addr);
      int client_fd = accept(server_fd, (sockaddr*)&client_addr, &len);
      if (client_fd < 0)
      {
        RCLCPP_WARN(this->get_logger(), "클라이언트 연결 실패");
        continue;
      }

      RCLCPP_INFO(this->get_logger(), "✅ 클라이언트 접속: %s", inet_ntoa(client_addr.sin_addr));

      std::thread([this, client_fd]() {
        while (tcp_running_)
        {
          std::string msg = get_robot_status_json();
          ssize_t sent1 = send(client_fd, msg.c_str(), msg.length(), 0);
          ssize_t sent2 = send(client_fd, "\n", 1, 0);

          if (sent1 <= 0 || sent2 <= 0)
          {
            RCLCPP_WARN(this->get_logger(), "❌ 연결 종료됨");
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        close(client_fd);
      }).detach();
    }
    close(server_fd);
  }
};

int main(int argc, char* argv[])
{
  signal(SIGPIPE, SIG_IGN);

  rclcpp::init(argc, argv);
  std::vector<RobotInfo> robots = {
    { "robot1", RobotType::MOBILE }, { "robot2", RobotType::MOBILE }, { "robot3", RobotType::MOBILE },
    { "robot4", RobotType::ARM },    { "robot5", RobotType::ARM },
  };
  auto node = std::make_shared<RobotStateManager>(robots);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
