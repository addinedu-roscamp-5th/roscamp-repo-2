#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <unordered_map>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <sstream>
#include <chrono>

#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <nlohmann/json.hpp>  // json 라이브러리 (https://github.com/nlohmann/json)

using json = nlohmann::json;

constexpr int TCP_PORT = 9999;

// ───── 상태 구조체 정의 ─────
struct RobotState
{
  float battery = -1.0f;
  geometry_msgs::msg::Pose pose;
  std::vector<double> joints;
};

class TcpGatewayNode : public rclcpp::Node
{
public:
  TcpGatewayNode(const std::vector<std::string>& robot_ids) : Node("tcp_gateway_node"), robot_ids_(robot_ids)
  {
    for (const auto& id : robot_ids_)
    {
      setup_subscribers(id);
    }
    tcp_thread_ = std::thread([this]() { this->run_tcp_server(); });
  }

  ~TcpGatewayNode()
  {
    tcp_running_ = false;
    if (tcp_thread_.joinable())
    {
      tcp_thread_.join();
    }
  }

private:
  std::vector<std::string> robot_ids_;
  std::unordered_map<std::string, RobotState> robot_states_;
  std::mutex state_mutex_;
  std::thread tcp_thread_;
  bool tcp_running_ = true;

  void setup_subscribers(const std::string& id)
  {
    std::string battery_topic = "/" + id + "/battery";
    create_subscription<std_msgs::msg::Float32>(battery_topic, 10,
                                                [this, id](const std_msgs::msg::Float32::SharedPtr msg) {
                                                  std::lock_guard<std::mutex> lock(state_mutex_);
                                                  robot_states_[id].battery = msg->data;
                                                });
    std::string pose_topic = "/" + id + "/camera_pose";
    create_subscription<geometry_msgs::msg::Pose>(pose_topic, 10,
                                                  [this, id](const geometry_msgs::msg::Pose::SharedPtr msg) {
                                                    std::lock_guard<std::mutex> lock(state_mutex_);
                                                    robot_states_[id].pose = *msg;
                                                    RCLCPP_INFO(this->get_logger(), "✅ camera_pose : %f %f %f", msg->position.x, msg->position.y, msg->position.z);
                                                  });

    std::string joint_topic = "/" + id + "/joint_states";
    create_subscription<sensor_msgs::msg::JointState>(joint_topic, 10,
                                                      [this, id](const sensor_msgs::msg::JointState::SharedPtr msg) {
                                                        std::lock_guard<std::mutex> lock(state_mutex_);
                                                        robot_states_[id].joints = msg->position;
                                                      });
  }

  std::string get_robot_status_json()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    json j;

    for (const auto& [id, state] : robot_states_)
    {
      j[id] = { { "battery", state.battery },
                { "pose",
                  { { "x", state.pose.position.x }, { "y", state.pose.position.y }, { "z", state.pose.position.z } } },
                { "joints", state.joints } };
    }
    return j.dump();
  }

  void run_tcp_server()
  {
    int server_fd, client_fd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_len = sizeof(client_addr);

    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == -1)
    {
      RCLCPP_ERROR(this->get_logger(), "소켓 생성 실패");
      return;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(TCP_PORT);

    if (bind(server_fd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "바인드 실패");
      close(server_fd);
      return;
    }

    // ✅ 바인드 성공 후 1회 JSON 전송
    std::string init_data = get_robot_status_json();
    RCLCPP_INFO(this->get_logger(), "🔔 바인드 성공 후 초기 로봇 상태 전송(JSON):\n%s", init_data.c_str());

    if (listen(server_fd, 1) < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "리스닝 실패");
      close(server_fd);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "✅ TCP 서버 대기 중 (PORT %d)...", TCP_PORT);

    while (tcp_running_)
    {
      client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_len);
      if (client_fd < 0)
      {
        RCLCPP_WARN(this->get_logger(), "클라이언트 연결 실패");
        continue;
      }

      RCLCPP_INFO(this->get_logger(), "✅ 클라이언트 접속: %s", inet_ntoa(client_addr.sin_addr));

      while (tcp_running_)
      {
        std::string data = get_robot_status_json();
        RCLCPP_INFO(this->get_logger(), "✅ 클라이언트 get_robot_status_json 전송: %s", data.c_str());
        send(client_fd, data.c_str(), data.length(), 0);
        send(client_fd, "\n", 1, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
      }

      close(client_fd);
    }

    close(server_fd);
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  std::vector<std::string> robot_ids = { "robot1", "robot2", "robot3", "robot4", "robot5" };

  auto node = std::make_shared<TcpGatewayNode>(robot_ids);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
