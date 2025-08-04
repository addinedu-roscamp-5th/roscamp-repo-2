#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <limits>
#include <chrono>
#include <memory>
#include <cmath>

using namespace std;

// ------------------- ENUMS -------------------
enum class TaskType { INBOUND, OUTBOUND, DISPLAY, PICK, CHARGE };
enum class RobotStatus { IDLE, BUSY, CHARGING, ERROR };

// ------------------- TASK -------------------
struct Task {
    string task_id;
    TaskType task_type;
    pair<int, int> location;
    int priority;
    chrono::system_clock::time_point deadline;
    string assigned_robot;
    string status;

    Task(string id, TaskType type, pair<int, int> loc, int prio = 1,
         chrono::system_clock::time_point dl = chrono::system_clock::time_point::max())
        : task_id(id), task_type(type), location(loc), priority(prio), deadline(dl),
          assigned_robot(""), status("대기") {}
};

// ------------------- ROBOT -------------------
struct Robot {
    string robot_id;
    RobotStatus status;
    pair<int, int> current_location;
    shared_ptr<Task> task;
    int total_tasks_handled;

    Robot(string id, pair<int, int> loc)
        : robot_id(id), status(RobotStatus::IDLE), current_location(loc),
          task(nullptr), total_tasks_handled(0) {}

    void assign_task(shared_ptr<Task> t) {
        task = t;
        status = RobotStatus::BUSY;
        total_tasks_handled++;
        t->assigned_robot = robot_id;
        t->status = "진행중";
    }
};

// ------------------- SCHEDULER -------------------
class TaskScheduler {
public:
    vector<shared_ptr<Task>> task_queue;
    vector<shared_ptr<Robot>> robots;

    void add_task(shared_ptr<Task> task) {
        task_queue.push_back(task);
        sort_tasks();
    }

    void register_robot(shared_ptr<Robot> robot) {
        robots.push_back(robot);
    }

    void schedule() {
        if (robots.empty()) {
            cout << "\u274C 로봇 없음" << endl;
            return;
        }

        sort_tasks();

        for (auto it = task_queue.begin(); it != task_queue.end();) {
            auto task = *it;
            vector<shared_ptr<Robot>> idle_robots;
            for (auto& r : robots) {
                if (r->status == RobotStatus::IDLE)
                    idle_robots.push_back(r);
            }
            if (idle_robots.empty()) {
                cout << "\u26A0\uFE0F 할당 실패: 모든 로봇이 바쁨" << endl;
                break;
            }

            // 복합 스코어 계산
            auto selected_robot = *min_element(idle_robots.begin(), idle_robots.end(),
                [&](const shared_ptr<Robot>& a, const shared_ptr<Robot>& b) {
                    return score(a, task->location, a->total_tasks_handled)
                         < score(b, task->location, b->total_tasks_handled);
                });

            selected_robot->assign_task(task);
            cout << "[복합할당] 작업 " << task->task_id << " → 로봇 " << selected_robot->robot_id << endl;
            it = task_queue.erase(it);
        }
    }

private:
    void sort_tasks() {
        sort(task_queue.begin(), task_queue.end(), [](const shared_ptr<Task>& a, const shared_ptr<Task>& b) {
            return tie(a->priority, a->deadline) < tie(b->priority, b->deadline);
        });
    }

    int distance(pair<int, int> a, pair<int, int> b) {
        return abs(a.first - b.first) + abs(a.second - b.second);
    }

    int score(shared_ptr<Robot> robot, pair<int, int> task_loc, int load) {
        return distance(robot->current_location, task_loc) + load * 5;
    }
};