#include <cmath>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <limits>
#include <fstream>
#include <sstream>
#include <deque>
#include <numeric>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp> 
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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
    
    // [추가된 변수들] 즉각적인 속도 측정을 위해 필요합니다.
    bool is_first_msg = true;     // 첫 메시지인지 확인용
    double prev_x = 0.0;          // 이전 X 좌표
    double prev_y = 0.0;          // 이전 Y 좌표
    rclcpp::Time last_time;       // 이전 메시지 수신 시간
    
    // 속도 노이즈 제거를 위한 버퍼 (최근 10개 데이터 평균)
    std::deque<double> vel_buffer;
    const size_t buffer_size = 10; 

    // 기존 변수들 (혹시 다른 곳에서 쓸 수도 있으니 유지, 필요 없으면 삭제 가능)
    double start_x = 0.0;         
    double start_y = 0.0;
    double end_x = 0.0;
    double end_y = 0.0;         
    bool is_initialized = false;    
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
  {2, {0, -0.4}}
//   {2, {0.1, -0.4}}
};

std::map<int, Pose> hv_rois = {
  {1, {0.9, 0.55}},
  {2, {1.1, -0.71}}
};

// Yellow ROI zones (속도 감속 구간)
std::map<int, Pose> yellow_cav_rois = {
  {1, {1.1945931911468506, 2.152200222015381}},     // CIRCLE 로타리 up ->DOWN  진입
  {2, {-0.4866666793823242, -0.53259406332969666}}, // CIRCLE 로타리 LEFT 진입
//   {3, {-3.738260269165039, -0.35879406332969666}},   // 2ND 사지교차로 LEFT 진입
  {3, {-3.518260269165039, -0.53259406332969666}},   // 2ND 사지교차로 LEFT 진입
  {4, {-2.2249999046325684, -1.3799999952316284}},   // 2ND 사지교차로 down-> up 진입 
  {5, {-2.4662363529205322, 1.4755114316940308}},   // 2ND 사지교차로 RIGHT 진입
  {6, {-3.6093127727508545, 2.3087010383605957}},   // IST RIGHT 진입
  {7, {-1.1506917476654053, -2.30232834815979}},   // 3RD RIGHT 진입
  {8, {-0.4866666793823242, +0.53259406332969666}}
};

// ========================= 
// Yellow Zone Groups (존별 속도 제어)
// =========================
// yellow_zone_group1: Zone 1,2 - 낮은 속도
// yellow_zone_group2: Zone 3,4,5,6,7 - 다른 속도
std::map<int, int> yellow_roi_to_zone_group = {
  {1, 1}, {3, 1},          // Zone 1,2 -> Group 1 // ->  zone3 append
  {2, 2}, {4, 2}, {5, 2}, {6, 2}, {7, 2}, {8, 2}  // Zone 3,4,5,6,7,8 -> Group 2
};

// Zone Group별 속도 설정
std::map<int, double> zone_group_speeds = {
  {1, 0.8},  // Group 1 (Zone 1,2): 0.8 m/s
  {2, 1.5}   // Group 2 (Zone 3,4,5,6,7): 1.5 m/s
};

// CAV별 현재 속한 zone_group 추적 (0 = 아무 zone도 아님)
std::map<int, int> cav_current_zone_group = {
  {1, 0}, {2, 0}, {3, 0}, {4, 0}
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

// =========================
// CAV의 현재 zone group 판단 함수
// =========================
int get_zone_group_for_cav(int cav_index, const Pose& cav_pose, double detection_radius) {
    for (const auto& [yellow_roi_id, yellow_roi_pose] : yellow_cav_rois) {
        double dist = calculate_distance(cav_pose, yellow_roi_pose);
        if (dist <= detection_radius) {
            // yellow_roi_to_zone_group에서 해당 zone group 반환
            return yellow_roi_to_zone_group[yellow_roi_id];
        }
    }
    return 0;  // 어떤 zone에도 속하지 않음
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
// Yellow Zone Visualization Function
// =========================
void publish_yellow_zone_markers(std::shared_ptr<rclcpp::Node> node, 
                                  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub,
                                  double yellow_roi_detection_radius) {
    auto marker_array = std::make_shared<visualization_msgs::msg::MarkerArray>();
    
    int marker_id = 0;
    for (const auto& [yellow_roi_id, yellow_roi_pose] : yellow_cav_rois) {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = node->now();
        marker.ns = "yellow_zones";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Set position
        marker.pose.position.x = yellow_roi_pose.x;
        marker.pose.position.y = yellow_roi_pose.y;
        marker.pose.position.z = 0.0;
        
        // Set orientation (no rotation needed for cylinder)
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        
        // Set scale (radius and height)
        marker.scale.x = yellow_roi_detection_radius * 2.0;  // diameter
        marker.scale.y = yellow_roi_detection_radius * 2.0;  // diameter
        marker.scale.z = 0.01;  // thin cylinder for top-down view
        
        // Set color: Yellow (RGB)
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.3f;  // semi-transparent
        
        marker_array->markers.push_back(marker);
        
        // Add text label
        auto text_marker = visualization_msgs::msg::Marker();
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = node->now();
        text_marker.ns = "yellow_zone_labels";
        text_marker.id = marker_id++;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;
        
        text_marker.pose.position.x = yellow_roi_pose.x;
        text_marker.pose.position.y = yellow_roi_pose.y;
        text_marker.pose.position.z = 0.2;
        
        text_marker.pose.orientation.w = 1.0;
        text_marker.scale.z = 0.15;  // text size
        
        text_marker.color.r = 1.0f;
        text_marker.color.g = 1.0f;
        text_marker.color.b = 0.0f;
        text_marker.color.a = 1.0f;
        
        text_marker.text = "Yellow_ROI_" + std::to_string(yellow_roi_id);
        marker_array->markers.push_back(text_marker);
    }
    
    marker_pub->publish(*marker_array);
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
                // RCLCPP_WARN(node->get_logger(), 
                //     ">>> ROI_%d: HV Arrived! Releasing CAV_%d (was waiting at ROI_%d) <<<", 
                //     hv_roi_id,   //  HV가 도착한 ROI
                //     cav_id, 
                //     cav_roi_id);
            } 
            else {
                msg.data = 1; // STOP
                vel_msg.data = 0.0;

                if(target_vel_pubs.count(cav_id)) target_vel_pubs[cav_id]->publish(vel_msg);
                if(red_flag_pubs.count(cav_id)) red_flag_pubs[cav_id]->publish(msg);
                // RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, 
                //     "ROI_%d: CAV_%d Stop", cav_roi_id, cav_id);
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
// [New Logic] 즉각적인 속도 측정 함수
// ==========================================
void CalculateInstantVelocity(const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                              int hv_id, 
                              std::map<int, HVState>& states) 
{
    HVState& state = states[hv_id];
    rclcpp::Time current_time = msg->header.stamp;
    double current_x = msg->pose.position.x;
    double current_y = msg->pose.position.y;

    // 1. 첫 메시지면 초기화만 하고 리턴
    if (state.is_first_msg) {
        state.prev_x = current_x;
        state.prev_y = current_y;
        state.last_time = current_time;
        state.is_first_msg = false;
        return;
    }

    // 2. 시간 변화량 (dt) 계산
    double dt = (current_time - state.last_time).seconds();

    // dt가 너무 작으면(0.02초 미만, 50Hz 기준) 계산 스킵 (Division by zero 방지 및 노이즈 감소)
    if (dt < 0.001) return; 

    // 3. 이동 거리 (dist) 계산
    double dx = current_x - state.prev_x;
    double dy = current_y - state.prev_y;
    double dist = std::hypot(dx, dy);

    // 4. 순간 속도 계산 (v = d / t)
    double current_vel = dist / dt;

    // 5. 이동 평균 필터 (Moving Average)
    // 버퍼에 현재 속도 추가
    if (state.vel_buffer.size() >= state.buffer_size) {
        state.vel_buffer.pop_front();
    }
    state.vel_buffer.push_back(current_vel);

    // 버퍼 평균 계산
    double sum = std::accumulate(state.vel_buffer.begin(), state.vel_buffer.end(), 0.0);
    double avg_vel = sum / state.vel_buffer.size();

    // 6. 전역 변수 업데이트 (혹은 필요한 곳에 사용)
    // 노이즈로 인한 미세한 움직임은 0으로 처리 (Optional)
    if (avg_vel < 0.01) avg_vel = 0.0;
    
    // 최대 속도 제한 (시뮬레이터 튀는 현상 방지)
    if (avg_vel > 2.5) avg_vel = 2.5;

    measured_hv_vel = avg_vel;

    // 7. 상태 업데이트
    state.prev_x = current_x;
    state.prev_y = current_y;
    state.last_time = current_time;

    // [Log] 확인용 (필요시 주석 해제)
    // RCLCPP_INFO(rclcpp::get_logger("hv_vel"), "HV_%d Speed: %.3f m/s", hv_id, measured_hv_vel);
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

    // Get TEAM_AIM_HOME environment variable (default: /root/TEAM_AIM)
    const char* team_aim_home = std::getenv("TEAM_AIM_HOME");
    std::string base_path = (team_aim_home != nullptr && strlen(team_aim_home) > 0) 
                            ? std::string(team_aim_home) 
                            : std::string("/root/TEAM_AIM");
    std::string path_dir = base_path + "/src/global_path/";

    RCLCPP_INFO(node->get_logger(), "Loading CSV path files...");
    int i = 19;
    std::string csv_path = path_dir + "path_mission3_" + std::to_string(i) + ".csv";
    hv_paths[i] = load_csv_file(csv_path);
    RCLCPP_INFO(node->get_logger(), "HV_%d: %zu waypoints loaded", i, hv_paths[i].size());
    

    // Get CAV_IDS from environment variable (format: "32,3,5,6")
    const char* cav_ids_env = std::getenv("CAV_IDS");
    std::vector<int> active_cav_ids;
    if (cav_ids_env != nullptr && strlen(cav_ids_env) > 0) {
        std::string cav_ids_str(cav_ids_env);
        std::stringstream ss(cav_ids_str);
        std::string token;
        while (std::getline(ss, token, ',')) {
            token.erase(0, token.find_first_not_of(" \t"));
            token.erase(token.find_last_not_of(" \t") + 1);
            if (!token.empty()) {
                active_cav_ids.push_back(std::stoi(token));
            }
        }
    } else {
        active_cav_ids = {1, 2, 3, 4};
    }

    // RED_FLAG & YELLOW_FLAG Publisher (based on CAV_IDS indices 1-4)
    std::map<int, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> red_flag_pubs;
    std::map<int, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> yellow_flag_pubs;
    std::map<int, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> target_vel_pubs;
    
    // Marker Publisher for yellow zone visualization
    auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("/yellow_zones_markers", 10);
    
    for (size_t i = 0; i < active_cav_ids.size() && i < 4; i++) {
        int cav_index = (int)i + 1;  // 1-indexed
        int actual_cav_id = active_cav_ids[i];  // actual CAV ID from CAV_IDS
        const std::string cav_id_str = std::string(2 - std::to_string(actual_cav_id).length(), '0') + std::to_string(actual_cav_id);
        const std::string flag_topic = "/CAV_" + cav_id_str + "_RED_FLAG";
        red_flag_pubs[cav_index] = node->create_publisher<std_msgs::msg::Int32>(flag_topic, 1);
        const std::string yellow_flag_topic = "/CAV_" + cav_id_str + "_YELLOW_FLAG";
        yellow_flag_pubs[cav_index] = node->create_publisher<std_msgs::msg::Int32>(yellow_flag_topic, 1);
        const std::string vel_topic = "/CAV_" + cav_id_str + "_target_vel";
        target_vel_pubs[cav_index] = node->create_publisher<std_msgs::msg::Float64>(vel_topic, 1);
        RCLCPP_INFO(node->get_logger(), "Created publishers for CAV_Index_%d (Actual_ID=%d): topics %s, %s, %s", cav_index, actual_cav_id, flag_topic.c_str(), yellow_flag_topic.c_str(), vel_topic.c_str());
    }

    // Subscribers for CAV indices (1-4), HV 19, 20
    std::vector<std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>>> cav_subscriptions;
    
    for (size_t i = 0; i < active_cav_ids.size() && i < 4; i++) {
        int cav_index = (int)i + 1;  // 1-indexed
        int actual_cav_id = active_cav_ids[i];  // actual CAV ID from CAV_IDS
        const std::string cav_id_str = std::string(2 - std::to_string(actual_cav_id).length(), '0') + std::to_string(actual_cav_id);
        std::string cav_topic = "/CAV_" + cav_id_str;
        auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
            cav_topic, rclcpp::SensorDataQoS(),
            [cav_index](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                cav_poses[cav_index] = {msg->pose.position.x, msg->pose.position.y};
            });
        cav_subscriptions.push_back(sub);
        RCLCPP_INFO(node->get_logger(), "Subscribed to CAV_Index_%d (Actual_ID=%d): %s", cav_index, actual_cav_id, cav_topic.c_str());
    }

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
            hv_19_callback(msg); 
            CalculateInstantVelocity(msg, 19, hv_states);
    });



    rclcpp::Rate r(50);
    std::map<int, std::vector<int>> prev_red_flag_vehicles;
    std::map<int, std::set<int>> cav_active_states;
    std::map<int, std::set<int>> cav_released_states;
    
    double yellow_roi_detection_radius = 0.5;  // yellow ROI 감지 반경 0.3 / 0.5 / 0.4
    RCLCPP_INFO(node->get_logger(), "Simple Speed Trap Started.");
    
    while(rclcpp::ok()) {
        // Publish yellow zone markers for visualization in RViz
        publish_yellow_zone_markers(node, marker_pub, yellow_roi_detection_radius);
        
        // 모든 ROI 쌍에 대해 모니터링 수행
    for (const auto& pair : roi_pairs) {
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

        // Yellow ROI 모니터링 - Zone Group별 플래그 발행
        for (const auto& [cav_index, cav_pose] : cav_poses) {
            int current_zone_group = get_zone_group_for_cav(cav_index, cav_pose, yellow_roi_detection_radius);
            int previous_zone_group = cav_current_zone_group[cav_index];
            
            // Zone group이 변경되었을 때
            if (current_zone_group != previous_zone_group) {
                cav_current_zone_group[cav_index] = current_zone_group;
                
                // yellow_flag_1 또는 yellow_flag_2 발행 (또는 0으로 리셋)
                auto yellow_msg = std_msgs::msg::Int32();
                yellow_msg.data = current_zone_group;  // 0, 1, or 2
                if (yellow_flag_pubs.count(cav_index)) {
                    yellow_flag_pubs[cav_index]->publish(yellow_msg);
                }
                
                if (current_zone_group == 0) {
                    RCLCPP_WARN(node->get_logger(), ">>> CAV_%d: Left Yellow Zone - Yellow Flag OFF <<<", cav_index);
                } else {
                    RCLCPP_WARN(node->get_logger(), ">>> CAV_%d: Entered Yellow Zone Group_%d - Yellow Flag = %d <<<", cav_index, current_zone_group, current_zone_group);
                }
            }
        }

        rclcpp::spin_some(node);
        r.sleep();
    }
    rclcpp::shutdown();
    return 0;
}