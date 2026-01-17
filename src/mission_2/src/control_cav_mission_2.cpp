#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <std_msgs/msg/int32.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <memory>
#include <algorithm>
#include <cstdlib>
#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <vector>
#include <map>
#include <chrono>
#include <filesystem>
#include <ament_index_cpp/get_package_prefix.hpp>

using namespace std;

// =========================
// Path data structure
// =========================
struct PathPoint {
    double x;
    double y;
};

// =========================
// Global variables
// =========================
static std::vector<PathPoint> lane_paths[4];  // lane_paths[1], [2], [3] for lanes 1,2,3
static std::map<int, int> lane_start_idx;     // lane_start_idx[1], [2], [3] = closest index per lane
static std::vector<PathPoint> integrate_path_vector;  // Current active path
static int current_lane = 2;  // Current active lane (1=LEFT, 2=CENTER, 3=RIGHT)
static bool fca_collision_flag = false; 

// CAV pose
static double cav_x = 0.0;
static double cav_y = 0.0;
static double cav_z = 0.0;
static double cav_yaw = 0.0;

// HV positions (ID -> (x, y))
static std::map<int, std::pair<double, double>> hv_positions;

// =========================
// Zone-based collision detection
// =========================
// Zone 1: HV ROI
static std::vector<PathPoint> hv_zone_polygon = {
    {3.5416667461395264, -1.8333686590194702},
    {3.537745475769043, -1.9098244905471802},
    {3.5260493755340576, -1.9686254262924194},
    {3.5105035305023193, -2.0161173343658447},
    {3.5028510093688965, -2.0345921516418457},
    {3.475167989730835, -2.08777117729187},
    {3.4365811347961426, -2.141998291015625},
    {3.4239115715026855, -2.1566786766052246},
    {3.3929789066314697, -2.1879186630249023},
    {3.333669900894165, -2.233895778656006},
    {3.24945068359375, -2.277170181274414},
    {3.1629221439361572, -2.301370143890381},
    {3.093331813812256, -2.3082242012023926},
    {3.020509958267212, -2.302333354949951}
};

// Zone 1: CAV ROI (선분)
static PathPoint cav_zone_start = {3.4, -2.549999952316284};
static PathPoint cav_zone_end   = {3.2733333110809326, -2.549999952316284};

// Zone 1 flag (복구!)
static bool zone_collision_flag = false;

// =========================
// Zone 2
// =========================
// CAV Zone 2: 수직 선분
static PathPoint cav_zone2_start = {5.058333396911621, 1.0199999809265137};
static PathPoint cav_zone2_end = {5.058333396911621, 0.6666666865348816};

// HV Zone 2: 수직 선분
static PathPoint hv_zone2_start = {5.308333396911621, 0.4};
static PathPoint hv_zone2_end = {5.308333396911621, 0.09};

// Zone 2 flag
static bool zone2_collision_flag = false;

// =========================
// Controller state
// =========================
struct ControllerState {
    double speed_mps{0.5};
    double lookahead_m{0.3};
    double max_yaw_rate{1.5};
    double prev_x{0.0};
    double prev_y{0.0};
    double prev_yaw{0.0};
    bool has_prev{false};
    int red_flag{0};
};

// =========================
// HV Velocity Measurement (추가!)
// =========================
struct HVState {
    int id;
    double start_x;
    double start_y;
    double end_x;
    double end_y;
    bool is_initialized{false};
    bool timer_running{false};
    rclcpp::Time start_time;
};

static std::map<int, HVState> hv_states;  // HV20, HV24 상태 저장
static std::map<int, std::vector<PathPoint>> hv_paths;  // HV20, HV24 경로
static double measured_hv20_vel = 1.3;  // Lane 3 (HV20) 기본 속도
static double measured_hv24_vel = 1.3;  // Lane 2 (HV24) 기본 속도

// =========================
// Helper utilities
// =========================
static std::string twoDigitId(int id) {
    if (id < 0) id = 0;
    if (id > 99) id = 99;
    std::ostringstream oss;
    oss << std::setw(2) << std::setfill('0') << id;
    return oss.str();
}

static double calculateDistance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::hypot(dx, dy);
}

static double yawFromQuat(const geometry_msgs::msg::Quaternion& qmsg, double fallback_yaw) {
    const double norm2 = qmsg.x*qmsg.x + qmsg.y*qmsg.y + qmsg.z*qmsg.z + qmsg.w*qmsg.w;
    if (norm2 <= 1e-6) return fallback_yaw;
    tf2::Quaternion q(qmsg.x, qmsg.y, qmsg.z, qmsg.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

static double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// =========================
// CSV loader
// =========================
static bool loadPathCsv(const std::string& csv_path, std::vector<PathPoint>& out) {
    std::ifstream file(csv_path);
    if (!file.is_open()) return false;

    out.clear();
    std::string line;

    // Skip header if exists
    if (std::getline(file, line)) {
        std::stringstream ss(line);
        double x, y;
        char comma;
        if (!(ss >> x >> comma >> y)) {
            // It's a header, skip
        } else {
            out.push_back({x, y});
        }
    }

    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        double x, y;
        char comma;

        if (!(ss >> x)) continue;
        if (!(ss >> comma)) continue;
        if (!(ss >> y)) continue;

        if (std::isnan(x) || std::isnan(y)) continue;
        out.push_back({x, y});
    }

    return out.size() >= 2;
}

// =========================
// Zone 1 Detection Functions (복구!)
// =========================

/// HV Zone 안에 HV가 있는지 체크 (각 점 기준 0.2m 반경)
bool is_hv_in_zone(double hv_x, double hv_y) {
    const double detection_radius = 0.2;

    for (const auto& zone_point : hv_zone_polygon) {
        double dist = calculateDistance(hv_x, hv_y, zone_point.x, zone_point.y);
        if (dist <= detection_radius) {
            return true;
        }
    }
    return false;
}

/// CAV Zone 안에 CAV가 있는지 체크 (선분으로부터 거리)
bool is_cav_in_zone(double cav_x_in, double cav_y_in) {
    const double detection_radius = 0.2;

    double x1 = cav_zone_start.x;
    double y1 = cav_zone_start.y;
    double x2 = cav_zone_end.x;
    double y2 = cav_zone_end.y;

    double line_len_sq = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

    if (line_len_sq < 1e-6) {
        return calculateDistance(cav_x_in, cav_y_in, x1, y1) <= detection_radius;
    }

    double t = ((cav_x_in - x1) * (x2 - x1) + (cav_y_in - y1) * (y2 - y1)) / line_len_sq;
    t = std::max(0.0, std::min(1.0, t));

    double closest_x = x1 + t * (x2 - x1);
    double closest_y = y1 + t * (y2 - y1);

    double dist = calculateDistance(cav_x_in, cav_y_in, closest_x, closest_y);
    return dist <= detection_radius;
}

/// Zone 1 충돌 검사: HV Zone에 HV 있고 + CAV Zone에 CAV 있으면 충돌
bool check_zone_collision() {
    bool cav_in_zone1 = is_cav_in_zone(cav_x, cav_y);
    if (!cav_in_zone1) {
        return false;
    }

    for (const auto& [hv_id, hv_pos] : hv_positions) {
        if (is_hv_in_zone(hv_pos.first, hv_pos.second)) {
            std::cout << "[ZONE_COLLISION] HV_" << hv_id << " in HV Zone + CAV in CAV Zone!" << std::endl;
            return true;
        }
    }
    return false;
}

// =========================
// Zone 2 Detection Functions (추가!)
// =========================

/// HV Zone 2 안에 HV가 있는지 체크 (선분으로부터 거리)
bool is_hv_in_zone2(double hv_x, double hv_y) {
    const double detection_radius = 0.2;

    double x1 = hv_zone2_start.x;
    double y1 = hv_zone2_start.y;
    double x2 = hv_zone2_end.x;
    double y2 = hv_zone2_end.y;

    double line_len_sq = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

    if (line_len_sq < 1e-6) {
        return calculateDistance(hv_x, hv_y, x1, y1) <= detection_radius;
    }

    double t = ((hv_x - x1) * (x2 - x1) + (hv_y - y1) * (y2 - y1)) / line_len_sq;
    t = std::max(0.0, std::min(1.0, t));

    double closest_x = x1 + t * (x2 - x1);
    double closest_y = y1 + t * (y2 - y1);

    double dist = calculateDistance(hv_x, hv_y, closest_x, closest_y);

    return dist <= detection_radius;
}

/// CAV Zone 2 안에 CAV가 있는지 체크 (선분으로부터 거리)
bool is_cav_in_zone2(double cav_x_in, double cav_y_in) {
    const double detection_radius = 0.2;

    double x1 = cav_zone2_start.x;
    double y1 = cav_zone2_start.y;
    double x2 = cav_zone2_end.x;
    double y2 = cav_zone2_end.y;

    double line_len_sq = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

    if (line_len_sq < 1e-6) {
        return calculateDistance(cav_x_in, cav_y_in, x1, y1) <= detection_radius;
    }

    double t = ((cav_x_in - x1) * (x2 - x1) + (cav_y_in - y1) * (y2 - y1)) / line_len_sq;
    t = std::max(0.0, std::min(1.0, t));

    double closest_x = x1 + t * (x2 - x1);
    double closest_y = y1 + t * (y2 - y1);

    double dist = calculateDistance(cav_x_in, cav_y_in, closest_x, closest_y);

    return dist <= detection_radius;
}

/// Zone 2 충돌 검사: HV Zone 2에 HV 있고 + CAV Zone 2에 CAV 있으면 충돌
bool check_zone2_collision() {
    bool cav_in_zone2 = is_cav_in_zone2(cav_x, cav_y);

    if (!cav_in_zone2) {
        return false;
    }

    for (const auto& [hv_id, hv_pos] : hv_positions) {
        if (is_hv_in_zone2(hv_pos.first, hv_pos.second)) {
            std::cout << "[ZONE2_COLLISION] HV_" << hv_id << " in HV Zone2 + CAV in CAV Zone2!" << std::endl;
            return true;
        }
    }

    return false;
}




// ============================================
/// HV 속도 측정 콜백 (rotary_tower 로직 기반)
// ============================================
/// HV 경로에서 가장 가까운 waypoint 찾기
int find_closest_hv_waypoint(const std::vector<PathPoint>& path, double x, double y) {
    if (path.empty()) return -1;

    double min_distance = 1e10;
    int closest_idx = 0;

    for (size_t i = 0; i < path.size(); i++) {
        double distance = calculateDistance(x, y, path[i].x, path[i].y);
        if (distance < min_distance) {
            min_distance = distance;
            closest_idx = static_cast<int>(i);
        }
    }
    return closest_idx;
}

/// HV 속도 측정 콜백 (rotary_tower 로직 기반)
void MeasureHVVelocity(const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                       int hv_id,
                       std::shared_ptr<rclcpp::Node> node) {

    // HV 경로가 없으면 리턴
    if (hv_paths.find(hv_id) == hv_paths.end() || hv_paths[hv_id].empty()) return;

    HVState& current_hv = hv_states[hv_id];

    // 초기화: 시작점과 끝점 설정
    if (!current_hv.is_initialized) {
        current_hv.id = hv_id;

        // 현재 HV 위치
        double hv_x = hv_positions[hv_id].first;
        double hv_y = hv_positions[hv_id].second;

        int start_idx = find_closest_hv_waypoint(hv_paths[hv_id], hv_x, hv_y) + 10;

        if (start_idx < 0 || start_idx + 60 >= (int)hv_paths[hv_id].size()) return;

        int end_idx = start_idx + 60;

        current_hv.start_x = hv_paths[hv_id][start_idx].x;
        current_hv.start_y = hv_paths[hv_id][start_idx].y;
        current_hv.end_x = hv_paths[hv_id][end_idx].x;
        current_hv.end_y = hv_paths[hv_id][end_idx].y;

        current_hv.is_initialized = true;
        RCLCPP_INFO(node->get_logger(), "[HV%d] Speed measurement points set", hv_id);
        return;
    }

    // 속도 측정
    rclcpp::Time current_time = msg->header.stamp;
    double hv_x = msg->pose.position.x;
    double hv_y = msg->pose.position.y;

    double dist_to_start = std::hypot(hv_x - current_hv.start_x, hv_y - current_hv.start_y);
    double dist_to_end = std::hypot(hv_x - current_hv.end_x, hv_y - current_hv.end_y);

    // 시작점 통과
    if (!current_hv.timer_running && dist_to_start < 0.1) {
        current_hv.start_time = current_time;
        current_hv.timer_running = true;
        RCLCPP_INFO(node->get_logger(), "[HV%d] Passed start point", hv_id);
    }

    // 끝점 통과 및 속도 계산
    if (current_hv.timer_running && dist_to_end < 0.1) {
        double elapsed = (current_time - current_hv.start_time).seconds();

        if (elapsed > 0.1) {  // 최소 시간 체크
            double section_distance = calculateDistance(
                current_hv.start_x, current_hv.start_y,
                current_hv.end_x, current_hv.end_y
            );

            double speed = section_distance / elapsed;

            // 속도 저장
            if (hv_id == 20) {
                measured_hv20_vel = speed;
                RCLCPP_INFO(node->get_logger(),
                    "[HV20 SPEED] %.3f m/s (Lane 3)", speed);
            } else if (hv_id == 24) {
                measured_hv24_vel = speed;
                RCLCPP_INFO(node->get_logger(),
                    "[HV24 SPEED] %.3f m/s (Lane 2)", speed);
            }

            current_hv.timer_running = false;
        }
    }

    // 타임아웃 (10초)
    if (current_hv.timer_running &&
        (current_time - current_hv.start_time).seconds() > 10.0) {
        current_hv.timer_running = false;
        RCLCPP_WARN(node->get_logger(), "[HV%d] Speed measurement timeout", hv_id);
    }
}
// =========================
// ROI Detection Functions
// =========================

int get_lane_start_idx(int lane_id, double cav_x, double cav_y) {
    if (lane_id < 1 || lane_id > 3) return -1;
    if (lane_paths[lane_id].empty()) return -1;

    // Search globally (simple) or logically to keep consistent
    // For simplicity in this logic, we search strictly closest
    int closest_idx = 0;
    double min_dist = 1e10;

    for (size_t i = 0; i < lane_paths[lane_id].size(); ++i) {
        double dist = calculateDistance(cav_x, cav_y, 
                                       lane_paths[lane_id][i].x, 
                                       lane_paths[lane_id][i].y);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }

    return closest_idx;
}

// =========================
// [NEW] Overlap Detection Function
// =========================
// Lane 2와 Lane 3가 현재 위치(cav_x, cav_y) 기준으로 겹쳐있는지 확인하는 함수
bool check_overlap_lane2_3(double x, double y) {
    // 데이터가 없으면 검사 불가
    if (lane_paths[2].empty() || lane_paths[3].empty()) return false;

    // 현재 위치에서 각 차선의 가장 가까운 인덱스 계산
    // (기존 get_lane_start_idx 함수 재사용)
    int idx2 = get_lane_start_idx(2, x, y);
    int idx3 = get_lane_start_idx(3, x, y);

    if (idx2 < 0 || idx3 < 0) return false;

    double p2_x = lane_paths[2][idx2].x;
    double p2_y = lane_paths[2][idx2].y;
    double p3_x = lane_paths[3][idx3].x;
    double p3_y = lane_paths[3][idx3].y;

    // 두 차선의 가장 가까운 점 사이의 거리 계산
    double separation = std::hypot(p2_x - p3_x, p2_y - p3_y);

    // 거리가 0.2m 미만이면 사실상 같은 경로(중복)로 판단
    return (separation < 0.2);
}

bool is_collision(int idx, const std::vector<PathPoint>& path, 
                 double threshold = 0.15) { 
    if (idx < 0 || idx >= (int)path.size()) return false;

    const PathPoint& point = path[idx];

    for (const auto& [hv_id, hv_pos] : hv_positions) {
        double dist = calculateDistance(point.x, point.y, hv_pos.first, hv_pos.second);
        if (dist <= threshold) {
            return true;
        }
    }
    return false;
}

bool check_lane_roi_collision(int lane_id, int start_idx, 
                             const std::vector<PathPoint>& path) {
    if (lane_id < 1 || lane_id > 3) return false;
    
    // Check points ahead
    std::vector<int> roi_offsets = {-20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80, 90};
    
    for (int offset : roi_offsets) {
        int check_idx = start_idx + offset;
        
        if (check_idx < 0 || check_idx >= (int)path.size()) {
            continue;
        }
        
        if (is_collision(check_idx, path)) {
            return true;
        }
    }
    return false;
}

bool should_stop(const std::vector<bool>& collision_list, int current_lane) {
    // Lane 1: Lane 2가 막히면 정지
    if (current_lane == 1) {
        return collision_list[1] && collision_list[2];
    }
    // Lane 3: Lane 2가 막히면 정지
    else if (current_lane == 3) {
        return collision_list[3] && collision_list[2];
    }
    // Lane 2: Lane 1과 Lane 3이 *모두* 막혀야 정지 (하나라도 뚫려있으면 회피 가능)
    else if (current_lane == 2) {
        return collision_list[2] && collision_list[1] && collision_list[3];
    }
    
    return false;
}


// APPEND
bool FCA_collision()
{
    // Placeholder for FCA collision detection logic
    // In a real implementation, this would check sensors or other data
    return false; // Default to no collision
}




//  


int choose_lane(const std::vector<bool>& collision_list, int current_lane) {
    std::vector<int> priority;
    priority.push_back(current_lane);  
    
    if (current_lane == 1) {
        priority.push_back(2);  
    } else if (current_lane == 2) {
        // Standard priority: Check 3 then 1.
        // BUT if collision_list[3] is artificially true (due to overlap),
        // the loop will skip 3 and pick 1.
        priority.push_back(3);  
        priority.push_back(1);  
    } else if (current_lane == 3) {
        priority.push_back(2);  
    }
    
    for (int lane : priority) {
        if (!collision_list[lane]) {
            return lane;
        }
    }
    return current_lane;
}

void change_csv_state(int cav_id, int new_lane, std::shared_ptr<rclcpp::Node> node) {
    if (new_lane < 1 || new_lane > 3) return;

    // [로직 추가] 경로 중복 시 Lane 3 -> Lane 2로 강제 변환
    bool is_overlap = check_overlap_lane2_3(cav_x, cav_y);
    
    if (is_overlap && (new_lane == 3 || new_lane == 2)) {
        RCLCPP_WARN(node->get_logger(), 
            "[OVERLAP] Path 2 & 3 merged. Forcing Lane 3 -> Lane 2 recognition.");
        new_lane = 2; // 강제로 2번 차선으로 인식하게 함
    }

    // 이미 해당 차선이면 리턴 (단, 위에서 new_lane이 2로 바뀌었으면 아래 로직을 탈 수 있음)
    if (new_lane == current_lane) return;

    // 경로 교체 및 상태 업데이트
    integrate_path_vector = lane_paths[new_lane];
    current_lane = new_lane;
    
    lane_start_idx[new_lane] = get_lane_start_idx(new_lane, cav_x, cav_y);

    RCLCPP_INFO(node->get_logger(), 
                "[LANE_SWITCH] Switched to Lane %d (size: %zu, start_idx: %d)", 
                new_lane, integrate_path_vector.size(), lane_start_idx[new_lane]);
}

// =========================
// Waypoint search
// =========================

int findClosestPointSimple(const std::vector<PathPoint>& path, double x, double y) {
    if (path.empty()) return -1;
    int best = 0;
    double min_d = 1e10;

    for (size_t i = 0; i < path.size(); i++) {
        double d = calculateDistance(x, y, path[i].x, path[i].y);
        if (d < min_d) {
            min_d = d;
            best = i;
        }
    }
    return best;
}

int findClosestPointAhead(const std::vector<PathPoint>& path, 
                         double x, double y, int ref_idx) {
    if (path.empty()) return -1;
    
    int start_idx = std::max(0, ref_idx);
    int best = start_idx;
    double min_d = 1e10;

    // Search only forward to prevent jitter
    int search_limit = std::min((int)path.size(), start_idx + 200);

    for (int i = start_idx; i < search_limit; i++) {
        double d = calculateDistance(x, y, path[i].x, path[i].y);
        if (d < min_d) {
            min_d = d;
            best = i;
        }
    }
    return best;
}

int findClosestPoint(const std::vector<PathPoint>& path, double x, double y, int lane) {
    if (lane_start_idx.find(lane) == lane_start_idx.end()) {
        lane_start_idx[lane] = findClosestPointSimple(path, x, y);
        return lane_start_idx[lane];
    }
    
    int result = findClosestPointAhead(path, x, y, lane_start_idx[lane]);
    lane_start_idx[lane] = result;
    return result;
}

int findWaypoint(const std::vector<PathPoint>& path, double x, double y, double lookahead) {
    int closest_idx = findClosestPointSimple(path, x, y);
    if (closest_idx < 0) return -1;

    for (int i = closest_idx; i < (int)path.size(); i++) {
        double d = calculateDistance(x, y, path[i].x, path[i].y);
        if (d > lookahead) {
            return i;
        }
    }

    return path.size() - 1;
}

bool isCorner(const std::vector<PathPoint>& path, int closest_idx) {
    int future_offset = 20;
    int path_size = (int)path.size();

    if (closest_idx + future_offset + 1 >= path_size) {
        return false;
    }

    double current_angle = atan2(path[closest_idx + 1].y - path[closest_idx].y,
                                 path[closest_idx + 1].x - path[closest_idx].x);
    
    int future_idx = closest_idx + future_offset;
    double future_angle = atan2(path[future_idx + 1].y - path[future_idx].y,
                               path[future_idx + 1].x - path[future_idx].x);
    
    double diff = future_angle - current_angle;
    diff = normalizeAngle(diff);
    diff = fabs(diff);
    if (diff > M_PI / 2) {
        diff = M_PI - diff;
    }

    double threshold_rad = (10.0 * M_PI / 180.0);
    return (diff > threshold_rad);
}

// [수정 후] stop_flag 인자 추가
void planVelocity(ControllerState& st, bool isCornerDetected, bool stop_flag) {
    
    // 1. 코너 주행 여부 확인
    if (isCornerDetected) {
        st.speed_mps = 1.0;
    } else {
        st.speed_mps = 1.5;
    }

    // 2. Stop Flag에 따른 속도 재조정 (Main 루프 하단에 있던 로직을 여기로 통합 추천)
    if (stop_flag) {
        if (current_lane == 2) {
            st.speed_mps = measured_hv24_vel; // Lane 2 막힘 -> 서행
        } else if (current_lane == 3) {
            st.speed_mps = measured_hv20_vel; // Lane 3 막힘 -> 약간 감속
        }
    }
    
    // 주의: ControllerState에 저장된 speed_mps는 나중에 wz(조향각) 계산과 cmd.linear.x에 사용됩니다.
}

void GetLd(ControllerState& st) {
    double gain_ld = 0.5;
    double max_ld = 0.35;
    double min_ld = 0.0;
    double ld = gain_ld * st.speed_mps;
    st.lookahead_m = std::max(min_ld, std::min(max_ld, ld));
}

// =========================
// Main
// =========================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("overtake");
    auto st = std::make_shared<ControllerState>();

    // ---- CAV ID setup
    int cav_id = 1;
    const char* env_id = std::getenv("CAV_ID");
    if (env_id) {
        try {
            cav_id = std::stoi(env_id); 
        } catch (...) {}
    }
    
    node->declare_parameter<int>("cav_id", cav_id);
    cav_id = node->get_parameter("cav_id").as_int();

    const std::string my_id_str = twoDigitId(cav_id);
    const std::string pose_topic = "/CAV_" + my_id_str;
    const std::string accel_topic = "/CAV_" + my_id_str + "_accel";
    const std::string red_flag_topic = "/CAV_" + my_id_str + "_RED_FLAG";

    RCLCPP_INFO(node->get_logger(), 
                "[INIT] CAV Mission 2 RE-REFACTORED started (CAV_%s)", my_id_str.c_str());

    // ---- Control parameters
    node->declare_parameter<double>("speed_mps", 1.5);
    node->declare_parameter<double>("lookahead_m", 0.3);
    node->declare_parameter<double>("max_yaw_rate", 4.5);

    st->speed_mps = node->get_parameter("speed_mps").as_double();
    st->lookahead_m = node->get_parameter("lookahead_m").as_double();
    st->max_yaw_rate = node->get_parameter("max_yaw_rate").as_double();

    // ---- Preload all lane paths
    RCLCPP_INFO(node->get_logger(), "[PRELOAD] Loading all lanes...");

    // Determine global_path directory: prefer installed package share if it exists,
    // otherwise fall back to repo global_path
    std::string global_path_prefix;
    try {
        std::string prefix = ament_index_cpp::get_package_prefix("mission_2");
        std::string candidate = prefix + "/share/mission_2/global_path/";
        if (std::filesystem::exists(candidate)) {
            global_path_prefix = candidate;
        } else {
            global_path_prefix = std::string("/root/TEAM_AIM/src/global_path/");
        }
    } catch (...) {
        global_path_prefix = std::string("/root/TEAM_AIM/src/global_path/");
    }

    for (int lane = 1; lane <= 3; ++lane) {
        std::string csv_path = global_path_prefix + "path_mission2_" + std::to_string(cav_id) + "_" + twoDigitId(lane) + ".csv";

        if (!loadPathCsv(csv_path, lane_paths[lane])) {
            RCLCPP_ERROR(node->get_logger(), "[PRELOAD] Failed to load lane %d (path=%s)", lane, csv_path.c_str());
            rclcpp::shutdown();
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "[PRELOAD] Lane %d loaded: %zu points (from %s)", 
                    lane, lane_paths[lane].size(), csv_path.c_str());
    }

    // HV 경로 로드 (추가!)
    RCLCPP_INFO(node->get_logger(), "[PRELOAD] Loading HV paths for velocity measurement...");

    // HV20 (Lane 3) 경로
    std::string hv20_path = global_path_prefix + std::string("path_mission2_1_20.csv");
    if (loadPathCsv(hv20_path, hv_paths[20])) {
        RCLCPP_INFO(node->get_logger(), "[PRELOAD] HV20 path loaded: %zu points (from %s)", hv_paths[20].size(), hv20_path.c_str());
    } else {
        RCLCPP_WARN(node->get_logger(), "[PRELOAD] Failed to load HV20 path (path=%s)", hv20_path.c_str());
    }

    // HV24 (Lane 2) 경로
    std::string hv24_path = global_path_prefix + std::string("path_mission2_1_24.csv");
    if (loadPathCsv(hv24_path, hv_paths[24])) {
        RCLCPP_INFO(node->get_logger(), "[PRELOAD] HV24 path loaded: %zu points (from %s)", hv_paths[24].size(), hv24_path.c_str());
    } else {
        RCLCPP_WARN(node->get_logger(), "[PRELOAD] Failed to load HV24 path (path=%s)", hv24_path.c_str());
    }
    

    // ---- Set initial path
    current_lane = 2;  // Start with CENTER lane
    integrate_path_vector = lane_paths[current_lane];
    lane_start_idx[current_lane] = 0;

    if (integrate_path_vector.empty()) {
        RCLCPP_FATAL(node->get_logger(), "[FATAL] Initial path is empty");
        rclcpp::shutdown();
        return 1;
    }

    // ---- Publishers & Subscribers
    auto accel_pub = node->create_publisher<geometry_msgs::msg::Accel>(accel_topic, rclcpp::SensorDataQoS());

    auto flag_sub = node->create_subscription<std_msgs::msg::Int32>(
        red_flag_topic, 10,
        [st](const std_msgs::msg::Int32::SharedPtr msg) {
            st->red_flag = msg->data;
        }
    );

    // ---- HV subscriptions
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> hv_subs;
    for (int hv_id = 19; hv_id <= 36; ++hv_id) {
        std::string hv_topic = "/HV_" + std::to_string(hv_id);
        auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
            hv_topic, rclcpp::SensorDataQoS(),
            [hv_id](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                hv_positions[hv_id] = {msg->pose.position.x, msg->pose.position.y};
            }
        );
        hv_subs.push_back(sub);
    }
    RCLCPP_INFO(node->get_logger(), "[INIT] Subscribed to HV_19~HV_36");


    // HV 속도 측정을 위한 별도 구독 (추가!)
    auto hv20_vel_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/HV_20", rclcpp::SensorDataQoS(),
        [node](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            MeasureHVVelocity(msg, 20, node);
        }
    );

    auto hv24_vel_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/HV_24", rclcpp::SensorDataQoS(),
        [node](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            MeasureHVVelocity(msg, 24, node);
        }
    );
    RCLCPP_INFO(node->get_logger(), "[INIT] HV velocity measurement enabled (HV20, HV24)");

    // ---- CAV pose subscription and control loop
    static int pose_callback_count = 0;
    auto last_log_time = std::chrono::steady_clock::now();
    const int LOG_INTERVAL_MS = 2000; // 로그 주기 단축

    auto pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic, rclcpp::SensorDataQoS(),
        [node, st, accel_pub, cav_id, &last_log_time](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            
            pose_callback_count++;

            // ===== 1. Update CAV pose =====
            cav_x = msg->pose.position.x;
            cav_y = msg->pose.position.y;
            cav_z = msg->pose.position.z;
            cav_yaw = yawFromQuat(msg->pose.orientation, st->prev_yaw);

            if (st->has_prev) {
                const double dx = cav_x - st->prev_x;
                const double dy = cav_y - st->prev_y;
                if (std::hypot(dx, dy) > 1e-4) {
                    cav_yaw = std::atan2(dy, dx);
                }
            }
            st->prev_x = cav_x;
            st->prev_y = cav_y;
            st->prev_yaw = cav_yaw;
            st->has_prev = true;

            // ===== 2. ZONE 1 충돌 검사 (복구!) =====
            zone_collision_flag = check_zone_collision();

            // ===== 2-2. ZONE 2 충돌 검사 =====
            zone2_collision_flag = check_zone2_collision();

            // ==== 2-3 . FCA 충돌 검사 (추가!) ====
            fca_collision_flag = FCA_collision();

            // ===== 3. Get closest index for each lane =====
            std::vector<int> lane_closest(4);
            for (int lane = 1; lane <= 3; ++lane) {
                lane_closest[lane] = get_lane_start_idx(lane, cav_x, cav_y);
            }

            // 4. Physical Collision Check (동일)
            vector<bool> lane_collision(4, false);
            for(int i=1; i<=3; ++i) {
                lane_collision[i] = check_lane_roi_collision(i, lane_closest[i], lane_paths[i]);
            }

            // =================================================================================
            // [수정됨] 중복 차선(Overlap) 감지 함수 호출
            // =================================================================================
            bool is_overlap = check_overlap_lane2_3(cav_x, cav_y);
            
            if (is_overlap) {
                // 1) Lane 3는 물리적으로 막힌 것으로 간주 (선택 방지)
                lane_collision[3] = true;

                // 2) [추가 기능] 만약 현재 내가 Lane 3라고 생각하고 있다면?
                //    -> 즉시 Lane 2로 상태를 정정한다 (change_csv_state 호출)
                if (current_lane == 3) {
                    // 여기서 2번 차선으로 변경 요청을 보내면, 
                    // change_csv_state 내부 로직에 의해 경로가 바뀌고 current_lane = 2가 됨
                    change_csv_state(cav_id, 2, node);
                }
            }

            // ===== 5. Check if should STOP =====
            bool stop_flag = should_stop(lane_collision, current_lane);
            
            // ===== 6. Choose passable lane =====
            // lane_collision[3]이 true이므로, current=2일 때 Lane 1을 선택하게 됨
            int next_lane = choose_lane(lane_collision, current_lane);

            // ===== 7. Lane switch if needed =====
            // Zone1/Zone2 충돌 없고, stop_flag 아니면 차선 변경
            if (!zone_collision_flag && !zone2_collision_flag && !stop_flag && next_lane != current_lane) {
                change_csv_state(cav_id, next_lane, node);
           }

            // ===== 8. Lane switch if needed =====
            double lane_start_distance = 1e10;
            if (lane_start_idx[current_lane] >= 0 && lane_start_idx[next_lane] >= 0) {
                 // 목표 차선까지의 거리 계산이 아니라, 차선 변경 안정성을 위해 
                 // 현재 위치가 경로상에 제대로 있는지 확인하는 용도
                 lane_start_distance = calculateDistance(
                     lane_paths[next_lane][lane_start_idx[next_lane]].x, 
                     lane_paths[next_lane][lane_start_idx[next_lane]].y, 
                     cav_x, cav_y
                 );
            }

            // 차선 변경 실행 (1.5m 이내에 목표 차선의 시작점이 있어야 변경 - 너무 멀면 오류 가능성 배제)
            if (!stop_flag && next_lane != current_lane) { 
                 change_csv_state(cav_id, next_lane, node);
            }

            // 9. Log Lane Status (수정됨: 상태 확인용)
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count();
            
            if (elapsed >= LOG_INTERVAL_MS) {
                std::cout << "\n=== [Lane Status] (CAV" << cav_id << ") ===" << std::endl;
                std::cout << "CAV Pos: (" << std::fixed << std::setprecision(2) << cav_x << ", " << cav_y << ")" << std::endl;
                
                if (is_overlap) {
                    std::cout << ">>> OVERLAP DETECTED! Forcing Lane 3 -> Lane 2 logic <<<" << std::endl;
                }

                for (int lane = 1; lane <= 3; ++lane) {
                    std::string status = lane_collision[lane] ? "X OCCUPIED" : "O FREE";
                    if (is_overlap && lane == 3) status += " (Overlap Force)";
                    std::cout << "  Lane " << lane << " : " << status << std::endl;
                }
                std::cout << "-> Current: " << current_lane << " -> Next: " << next_lane << std::endl;
                last_log_time = now;
            }

            // ===== 10. Pure Pursuit =====
            int closest_idx = findClosestPoint(integrate_path_vector, cav_x, cav_y, current_lane);

            if (closest_idx < 0) return;

            bool corner = isCorner(integrate_path_vector, closest_idx);
            planVelocity(*st, corner, stop_flag);
            GetLd(*st);

            int target_idx = findWaypoint(integrate_path_vector, cav_x, cav_y, st->lookahead_m);
            if (target_idx < 0) target_idx = closest_idx; // fallback

            // Check bounds
            if (target_idx >= (int)integrate_path_vector.size()) target_idx = integrate_path_vector.size() - 1;

            const double dx = integrate_path_vector[target_idx].x - cav_x;
            const double dy = integrate_path_vector[target_idx].y - cav_y;

            const double cy = std::cos(cav_yaw);
            const double sy = std::sin(cav_yaw);
            const double x_v = cy * dx + sy * dy;
            const double y_v = -sy * dx + cy * dy;
            const double pp_dist = std::max(1e-3, std::hypot(x_v, y_v));
            const double kappa = (2.0 * y_v) / (pp_dist * pp_dist);

            double wz = st->speed_mps * kappa;
            wz = std::clamp(wz, -st->max_yaw_rate, st->max_yaw_rate);

            // ===== 10. Publish Control =====
            geometry_msgs::msg::Accel cmd;
            // aappend collision! 
            if (fca_collision_flag){
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                RCLCPP_WARN(node->get_logger(), "[FCA_COLLISION] Emergency Stop Activated!");
                fca_collision_flag = false; // Reset after handling
            }
            else{
                if (zone_collision_flag || zone2_collision_flag) {
                    cmd.linear.x = 0.0;
                    cmd.angular.z = wz;
                }
                else if (stop_flag && current_lane == 2) {
                    cmd.linear.x = measured_hv24_vel;
                    cmd.angular.z = wz;
                }
                else if (stop_flag && current_lane == 3) {
                    cmd.linear.x = measured_hv20_vel;
                    cmd.angular.z = wz;
                }
                else if (stop_flag && current_lane == 1) {
                    cmd.linear.x = 0.0;
                    cmd.angular.z = wz;
                }
                else {
                    cmd.linear.x = st->speed_mps;
                    cmd.angular.z = wz;
                }
            }   
            // unused fields
            cmd.linear.y = 0.0; cmd.linear.z = 0.0;
            cmd.angular.x = 0.0; cmd.angular.y = 0.0;

            accel_pub->publish(cmd);
        }
    );

    (void)flag_sub;
    (void)pose_sub;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}