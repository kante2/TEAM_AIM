#include <cmath>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <limits>
#include <fstream>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp> 
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace std;

// =========================
// 구조체 정의
// =========================
struct Pose {
  double x;
  double y;
};

struct HVState {
    int id;                 
    double start_x;         
    double start_y;
    double end_x;
    double end_y;         
    bool is_initialized;    
};


// =========================
// Global Variables
// =========================
std::map<int, Pose> cav_poses;
std::map<int, Pose> hv_poses;
std::map<int, std::vector<Pose>> cav_paths;
std::map<int, std::vector<Pose>> hv_paths;
std::map<int, HVState> hv_states;

// =========================
// 4개 ROI 정의 (반지름 0.075m)
// =========================
std::map<int, Pose> cav_rois = {
  {1, {1.43333333333, 1.5}},
  {2, {0.1, -0.4}}
};

std::map<int, Pose> hv_rois = {
  {1, {0.9, 0.55}},
  {2, {1.1, -0.71}}
};

// Key: ROI_ID, Value: 허가받은 차량 ID 목록
std::map<int, std::set<int>> roi_permit_vehicles;

double measured_hv_vel = 0.0;

// ==========================================
// 측정할 두 지점 좌표
// ==========================================
const double CENTER_X = 1.666666667; 
const double CENTER_Y = 0.0;



// =========================
// CAV, HV Pose 콜백 함수들
// =========================
void cav_01_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    cav_poses[1] = {msg->pose.position.x, msg->pose.position.y};
}

void cav_02_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    cav_poses[2] = {msg->pose.position.x, msg->pose.position.y};
}

void cav_03_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    cav_poses[3] = {msg->pose.position.x, msg->pose.position.y};
}

void cav_04_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    cav_poses[4] = {msg->pose.position.x, msg->pose.position.y};
}


void hv_19_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    hv_poses[19] = {msg->pose.position.x, msg->pose.position.y};
    // hv_velocity(hv19_state, msg);
}

void hv_20_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    hv_poses[20] = {msg->pose.position.x, msg->pose.position.y};
    // hv_velocity(hv20_state, msg);
}


// =========================
// CSV 파일 로드 함수
// =========================
std::vector<Pose> load_csv_file(const std::string& file_path) {
  std::vector<Pose> csv_data;
  std::ifstream file(file_path);
  
  if (!file.is_open()) {
    std::cerr << "[WARN] Failed to open: " << file_path << std::endl;
    return csv_data;
  }

  std::string line;
  int line_num = 0;
  
  while (std::getline(file, line)) {
    line_num++;
    
    if (line.empty() || line.find_first_not_of(" \t\r\n") == std::string::npos) {
      continue;
    }
    
    if (line_num == 1 && (line.find("x,y") != std::string::npos || 
                          line.find("X,Y") != std::string::npos)) {
      continue;
    }
    
    std::stringstream ss(line);
    std::string x_str, y_str;

    if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',')) {
      try {
        x_str.erase(0, x_str.find_first_not_of(" \t\r\n"));
        x_str.erase(x_str.find_last_not_of(" \t\r\n") + 1);
        y_str.erase(0, y_str.find_first_not_of(" \t\r\n"));
        y_str.erase(y_str.find_last_not_of(" \t\r\n") + 1);
        
        Pose pose;
        pose.x = std::stod(x_str);
        pose.y = std::stod(y_str);
        csv_data.push_back(pose);
      } catch (...) {
        continue;
      }
    }
  }
  
  return csv_data;
}


// ==========================================
// 호 길이 계산 함수
// ==========================================
double GetArcLength(double x1, double y1, double x2, double y2,
                      double cx, double cy) {
    double r1 = std::hypot(x1 - cx, y1 - cy);
    double r2 = std::hypot(x2 - cx, y2 - cy);
    double radius = (r1 + r2) / 2.0;

    double theta1 = atan2(y1 - cy, x1 - cx);
    double theta2 = atan2(y2 - cy, x2 - cx);
    
    double delta_theta = fabs(theta2 - theta1);


    double arc = radius * delta_theta; 
    return arc;
}

// =========================
// 거리 계산 함수
// =========================
double calculate_distance(Pose pose1, Pose pose2) {
    double dx = pose1.x - pose2.x;
    double dy = pose1.y - pose2.y;
    return std::hypot(dx, dy);
}

int find_closest_waypoint_index(const std::vector<Pose>& path, Pose current_pose) {
  if (path.empty()) return -1;

  double min_distance = std::numeric_limits<double>::max();
  int closest_idx = 0;

  for (size_t i = 0; i < path.size(); i++) {
    const double distance = calculate_distance(path[i], current_pose);
    if (distance < min_distance) {
      min_distance = distance;
      closest_idx = static_cast<int>(i);
    }
  }
  return closest_idx;
}

// =========================
// 통합 제어 함수 Pair 기반
// =========================

void monitor_all_rois(
    int cav_roi_id, int hv_roi_id,
    std::shared_ptr<rclcpp::Node> node,
    std::map<int, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr>& red_flag_pubs,
    std::map<int, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>& target_vel_pubs,
    double detection_radius,  // cav_roi에서는 저 범위 안에 들어오면 cav 멈추게, hv_roi에서는 hv가 통과할 때 cav에게 주행권한 주는 범위
    double reset_radius
) {
    Pose cav_origin = cav_rois[cav_roi_id];
    Pose hv_origin = hv_rois[hv_roi_id];
    
    // 현재 검사 중인 ROI의 허가증 목록만 가져옴
    std::set<int>& permit_list = roi_permit_vehicles[cav_roi_id];

    for (const auto& [cav_id, pose] : cav_poses) {
        
        double dist = calculate_distance(pose, cav_origin);

        auto msg = std_msgs::msg::Int32();
        auto vel_msg = std_msgs::msg::Float64();
        bool send_commend = false;
        bool has_permission = permit_list.count(cav_id);

        // 1. 리셋 로직 (버퍼 벗어나면)
        if (dist > reset_radius) {
            if (has_permission) {
                permit_list.erase(cav_id); // 내 구역 허가증만 회수

                vel_msg.data = -1.0; // 속도 명령 리셋 신호
                
                if(target_vel_pubs.count(cav_id)) target_vel_pubs[cav_id]->publish(vel_msg);
                
            }
            continue; 
        }


        // (Case A) 내 구역 허가증이 있는지 체크
        if (has_permission) {
            msg.data = 0; // GO
            
            send_commend = true;
            if(target_vel_pubs.count(cav_id)) target_vel_pubs[cav_id]->publish(vel_msg);
            if(red_flag_pubs.count(cav_id)) red_flag_pubs[cav_id]->publish(msg);
        }
        
        // (Case B) 허가증 없으면 감지 및 검사
        else if (dist <= detection_radius) {
            bool hv_here = false;
            for (const auto& [hv_id, hv_pose] : hv_poses) {
                if (calculate_distance(hv_pose, hv_origin) <= detection_radius) {
                    hv_here = true;
                    break;
                }
            }

            if (hv_here) {
                // HV 도착 -> 내 구역에 허가증 발급
                permit_list.insert(cav_id); 
                msg.data = 0;
                send_commend = true;

                if(target_vel_pubs.count(cav_id)) target_vel_pubs[cav_id]->publish(vel_msg);
                if(red_flag_pubs.count(cav_id)) red_flag_pubs[cav_id]->publish(msg);
                RCLCPP_WARN(node->get_logger(), 
                    ">>> ROI_%d: HV Arrived! Releasing CAV_%d (was waiting at ROI_%d) <<<", 
                    hv_roi_id,   //  HV가 도착한 ROI
                    cav_id, 
                    cav_roi_id);
            } 
            else {
                msg.data = 1; // STOP
                vel_msg.data = 0.0;

                if(target_vel_pubs.count(cav_id)) target_vel_pubs[cav_id]->publish(vel_msg);
                if(red_flag_pubs.count(cav_id)) red_flag_pubs[cav_id]->publish(msg);
                RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, 
                    "ROI_%d: CAV_%d Stop", cav_roi_id, cav_id);
            }
        }
        // 2. 명령 발행
        if (send_commend) {
            vel_msg.data = measured_hv_vel;
            
            if(red_flag_pubs.count(cav_id)) red_flag_pubs[cav_id]->publish(msg);
            if(target_vel_pubs.count(cav_id)) target_vel_pubs[cav_id]->publish(vel_msg);

        }
    }
}


// ==========================================
// [Callback] 속도 측정 ,,HV 19  하나로 측정함
// ==========================================
void GetVelocityCB(const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                    int hv_id, 
                    std::map<int, HVState>& states, 
                    const std::map<int, vector<Pose>>& paths) 
{
    if (paths.find(hv_id) == paths.end() || paths.at(hv_id).empty()) return;

    HVState& current_hv = states[hv_id];

    if (!current_hv.is_initialized) {
        int start_idx = find_closest_waypoint_index(paths.at(hv_id), hv_poses[hv_id]) + 10;
        if (start_idx < 0 || start_idx + 20 >= (int)paths.at(hv_id).size()) return;

        int end_idx = start_idx + 60;
        current_hv.start_x = paths.at(hv_id)[start_idx].x;
        current_hv.start_y = paths.at(hv_id)[start_idx].y;
        current_hv.end_x = paths.at(hv_id)[end_idx].x;
        current_hv.end_y = paths.at(hv_id)[end_idx].y;

        current_hv.is_initialized = true;
        std::cout << "[HV " << hv_id << "] Start Point Set." << std::endl;
        return;
    }
    rclcpp::Time current_msg_time = msg->header.stamp;

    static bool timer_running = false; 
    static rclcpp::Time start_time; 
    
    // 구간 거리 계산 (처음 한 번만 실행함)
    static const double SECTION_DISTANCE = GetArcLength(current_hv.start_x, current_hv.start_y, current_hv.end_x, current_hv.end_y, CENTER_X, CENTER_Y);
    static bool is_initialized = false;
    static const double radius = std::hypot(current_hv.start_x - CENTER_X, current_hv.start_y - CENTER_Y);
    if(!is_initialized) {
        RCLCPP_INFO(rclcpp::get_logger("init"), "Section Distance: %.3f m", SECTION_DISTANCE);
        is_initialized = true;
    }

    double hv_x = msg->pose.position.x;
    double hv_y = msg->pose.position.y;
    
    // 현재 위치와 목표점 사이의 거리 계산 (단순 거리)
    double dist_to_p1 = std::hypot(hv_x - current_hv.start_x, hv_y - current_hv.start_y);
    double dist_to_p2 = std::hypot(hv_x - current_hv.end_x, hv_y - current_hv.end_y);


    // 1. 시작 지점 (P1) 통과 감지
    if (dist_to_p1 < 0.05) {
        start_time = current_msg_time; 
        timer_running = true;
        RCLCPP_INFO(rclcpp::get_logger("speed_trap"), ">>> PASSED START POINT (P1) <<<");
    }

    // 2. 종료 지점 (P2) 통과 감지 & 속도 계산
    if (timer_running && dist_to_p2 < 0.05) {
        double elapsed = (current_msg_time - start_time).seconds();

        // P1과 P2가 겹치는 오작동 방지를 위해 최소 0.5초 경과 확인
        if (elapsed > 0.05) {
            double speed_x = SECTION_DISTANCE / elapsed;
            double speed_w = speed_x / radius; // 각속도 계산

            if (speed_x > 1.8) {
              measured_hv_vel = 1.8;
            }
            else {
              measured_hv_vel = speed_x;
            }

            
            RCLCPP_INFO(rclcpp::get_logger("speed_trap"), 
                "\n================ RESULT ================\n" 
                " Speed_x: %.3f m/s\n"
                " Speed_w: %.3f rad/s\n"
                " Time : %.3f sec\n"
                " Dist : %.3f m\n"
                "========================================\n", 
                speed_x, speed_w, elapsed, SECTION_DISTANCE);
            
            timer_running = false; // 측정 완료 후 리셋
        }
    }
    
    // 타임아웃 (10초 지나면 자동 리셋하는거임,, 차가 멈췄거나 경로 이탈 시)
    if (timer_running && (current_msg_time - start_time).seconds() > 10.0) {
        timer_running = false;
        RCLCPP_WARN(rclcpp::get_logger("speed_trap"), "Measurement Timeout. Reset.");
    }
}




int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("simple_speed_trap_node");

    double detection_radius = 0.4;
    double resert_radius = 1.0;

    std::vector<std::pair<int, int>> roi_pairs = {
        {1, 1}, // CAV ROI 1 <-> HV ROI 1
        {2, 2}  // CAV ROI 2 <-> HV ROI 2
    };

    std::string path_dir = "/root/TEAM_AIM/src/global_path/";

    RCLCPP_INFO(node->get_logger(), "Loading CSV path files...");
    int i = 19;
    std::string csv_path = path_dir + "path_mission3_" + std::to_string(i) + ".csv";
    hv_paths[i] = load_csv_file(csv_path);
    RCLCPP_INFO(node->get_logger(), "CAV_%d: %zu waypoints loaded", i, hv_paths[i].size());
    

      // RED_FLAG Publisher (CAV 1~4)
    std::map<int, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> red_flag_pubs;
    std::map<int, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> target_vel_pubs;
    for (int i = 1; i <= 4; i++) {
      const std::string flag_topic = "/CAV_0" + std::to_string(i) + "_RED_FLAG";
      red_flag_pubs[i] = node->create_publisher<std_msgs::msg::Int32>(flag_topic, 50);
      const std::string vel_topic = "/CAV_0" + std::to_string(i) + "_target_vel";
      target_vel_pubs[i] = node->create_publisher<std_msgs::msg::Float64>(vel_topic, 50);
    }


    // Subscribers for CAV 1~4, HV 19, 20
    auto sub1 = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/CAV_01", rclcpp::SensorDataQoS(),
      [node](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        cav_01_callback(msg);
    });

    auto sub2 = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/CAV_02", rclcpp::SensorDataQoS(),
      [node](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        cav_02_callback(msg);
    });

    auto sub3 = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/CAV_03", rclcpp::SensorDataQoS(),
      [node](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        cav_03_callback(msg);
    });

    auto sub4 = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/CAV_04", rclcpp::SensorDataQoS(),
      [node](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        cav_04_callback(msg);
    });

    auto sub19 = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/HV_19", rclcpp::SensorDataQoS(),
        [node](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          hv_19_callback(msg);
    });

    auto sub20 = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/HV_20", rclcpp::SensorDataQoS(),
        [node](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          hv_20_callback(msg);
    });

    auto sub_hv19 = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/HV_19", rclcpp::SensorDataQoS(),
        [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg){
            GetVelocityCB(msg, 19, hv_states, hv_paths);
    });
        



    rclcpp::Rate r(50);
    std::map<int, std::vector<int>> prev_red_flag_vehicles;
    std::map<int, std::set<int>> cav_active_states;
    std::map<int, std::set<int>> cav_released_states;
    RCLCPP_INFO(node->get_logger(), "Simple Speed Trap Started.");
    
    while(rclcpp::ok()) {
        // 모든 ROI 쌍에 대해 모니터링 수행
    for (const auto& pair : roi_pairs) {
            int cav_roi_id = pair.first;
            int hv_roi_id = pair.second;
            
            // 수정된 함수 호출
            monitor_all_rois(pair.first,        // cav_roi_id
                pair.second,       // hv_roi_id
                node, 
                red_flag_pubs,
                target_vel_pubs,
                detection_radius,   // double radius
                resert_radius
            );
        }

        rclcpp::spin_some(node);
        r.sleep();
    }
    rclcpp::shutdown();
    return 0;
}