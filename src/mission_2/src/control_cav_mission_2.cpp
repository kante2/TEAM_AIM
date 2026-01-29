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
#include <deque>      // [추가] 이동 평균 필터용
#include <numeric>    // [추가] std::accumulate용

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

// CAV pose
static double cav_x = 0.0;
static double cav_y = 0.0;
static double cav_z = 0.0;
static double cav_yaw = 0.0;

// HV positions (ID -> (x, y))
static std::map<int, std::pair<double, double>> hv_positions;

// =========================
// Overlap Region Globals
// =========================
static int overlap_start_idx = -1; // 겹침 시작 인덱스 (Lane 3 기준)
static int overlap_end_idx = -1;   // 겹침 끝 인덱스 (Lane 3 기준)
static bool overlap_detected = false;

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
static PathPoint cav_zone_start = {4.5, -2.549999952316284};
static PathPoint cav_zone_end   = {3.6, -2.549999952316284};

// Zone 1 flag (복구!)
static bool zone_collision_flag = false;
static bool zone_cav_flag = false;  // CAV가 Zone 1 안에 있는지 여부

// =========================
// Zone 2
// =========================
// CAV Zone 2: 수직 선분
static PathPoint cav_zone2_start = {5.058333396911621, 1.0199999809265137};
static PathPoint cav_zone2_end = {5.058333396911621, -0.3};

// HV Zone 2: 수직 선분
static PathPoint hv_zone2_start = {5.308333396911621, 1.0};
static PathPoint hv_zone2_end = {5.308333396911621, -0.5};

// Zone 2 flag
static bool zone2_collision_flag = false;

// =========================
// Zone 3 & 4 & 5
// =========================

// CAV Zone 3
static PathPoint cav_zone3_start = {3.6, 2.55};
static PathPoint cav_zone3_end = {4.41333333333333, 2.55};
static bool zone3_collision_flag = false;

// CAV Zone 3_1
static PathPoint cav_zone3_1_start = {2.68333333333333, 2.30833333333333};
static PathPoint cav_zone3_1_end = {4.41333333333333, 2.30833333333333};
static bool zone3_1_collision_flag = false;

// CAV Zone 3_2
static PathPoint cav_zone3_2_start = {2.0, 2.55};
static PathPoint cav_zone3_2_end = {3.6, 2.55};
static bool zone3_2_collision_flag = false;

// CAV Zone 4
static PathPoint cav_zone4_start = {5.05833333333333, 2.30833333333333};
static PathPoint cav_zone4_end = {5.05833333333333, 1.74};
static bool zone4_collision_flag = false;


// CAV Zone 5
static PathPoint cav_zone5_start = {5.29166666666667, 2.30833333333333};
static PathPoint cav_zone5_end = {5.29166666666667, -0.27};
static bool zone5_collision_flag = false;

// CAV Zone 6
static PathPoint cav_zone6_start = {3.10, 2.3083338737487793};
static PathPoint cav_zone6_end = {3.26, -2.3083338737487793};

static PathPoint hv_zone6_start = {3.5416667461395264, 2.308333396911621};
static PathPoint hv_zone6_end = {3.5416667461395264, 1.7899999618530273};
static bool zone6_collision_flag = false;

// =========================
// Controller state
// =========================
struct ControllerState {
    double speed_mps{0.5};
    double lookahead_m{0.3};
    double max_yaw_rate{4.5};
    double prev_x{0.0};
    double prev_y{0.0};
    double prev_yaw{0.0};
    bool has_prev{false};
    int red_flag{0};
};

struct CavState {
    int id;
    double start_x;
    double start_y;
    double start_yaw;
    int current_lap;
    bool is_initialized;
    bool is_in_line;
    bool is_finished;
};

// =========================
// HV Velocity Measurement (수정됨: 즉시 측정 방식)
// =========================
struct HVState {
    int id;
    
    // 즉각적인 속도 측정을 위한 변수
    bool is_first_msg = true;
    double prev_x = 0.0;
    double prev_y = 0.0;
    rclcpp::Time last_time;
    
    // 이동 평균 필터용 버퍼
    std::deque<double> vel_buffer;
    const size_t buffer_size = 10; // 최근 10개 평균
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
    const double detection_radius = 0.1;

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
    const double detection_radius = 0.1;

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

    if (dist <= detection_radius) {
        cout << "[ZONE1_DETECTION] CAV in CAV Zone1!" << std::endl;
        return true;
    }

    return false;
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
    const double detection_radius = 0.1;

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
    const double detection_radius = 0.1;

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

// =========================
// Zone 3 & 4 & 5 Detection Functions (실선 구간 차선 변경 불가 로직 추가하기 위함)
// =========================
/// CAV Zone 3 안에 CAV가 있는지 체크 (선분으로부터 거리)
bool is_cav_in_zone3(double cav_x_in, double cav_y_in) {
    const double detection_radius = 0.2;

    double x1 = cav_zone3_start.x;
    double y1 = cav_zone3_start.y;
    double x2 = cav_zone3_end.x;
    double y2 = cav_zone3_end.y;

    double line_len_sq = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

    if (line_len_sq < 1e-6) {
        return calculateDistance(cav_x_in, cav_y_in, x1, y1) <= detection_radius;
    }

    double t = ((cav_x_in - x1) * (x2 - x1) + (cav_y_in - y1) * (y2 - y1)) / line_len_sq;
    t = std::max(0.0, std::min(1.0, t));

    double closest_x = x1 + t * (x2 - x1);
    double closest_y = y1 + t * (y2 - y1);

    double dist = calculateDistance(cav_x_in, cav_y_in, closest_x, closest_y);

    if (dist <= detection_radius) {
        cout << "[ZONE3_DETECTION] CAV in CAV Zone3!" << std::endl;
        return true;
    }

    return false;
}

bool is_cav_in_zone3_1(double cav_x_in, double cav_y_in) {
    const double detection_radius = 0.2;

    double x1 = cav_zone3_1_start.x;
    double y1 = cav_zone3_1_start.y;
    double x2 = cav_zone3_1_end.x;
    double y2 = cav_zone3_1_end.y;

    double line_len_sq = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

    if (line_len_sq < 1e-6) {
        return calculateDistance(cav_x_in, cav_y_in, x1, y1) <= detection_radius;
    }

    double t = ((cav_x_in - x1) * (x2 - x1) + (cav_y_in - y1) * (y2 - y1)) / line_len_sq;
    t = std::max(0.0, std::min(1.0, t));

    double closest_x = x1 + t * (x2 - x1);
    double closest_y = y1 + t * (y2 - y1);

    double dist = calculateDistance(cav_x_in, cav_y_in, closest_x, closest_y);

    if (dist <= detection_radius) {
        cout << "[ZONE3_1_DETECTION] CAV in CAV Zone3_1!" << std::endl;
        return true;
    }

    return false;
}

bool is_cav_in_zone3_2(double cav_x_in, double cav_y_in) {
    const double detection_radius = 0.2;

    double x1 = cav_zone3_2_start.x;
    double y1 = cav_zone3_2_start.y;
    double x2 = cav_zone3_2_end.x;
    double y2 = cav_zone3_2_end.y;

    double line_len_sq = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

    if (line_len_sq < 1e-6) {
        return calculateDistance(cav_x_in, cav_y_in, x1, y1) <= detection_radius;
    }

    double t = ((cav_x_in - x1) * (x2 - x1) + (cav_y_in - y1) * (y2 - y1)) / line_len_sq;
    t = std::max(0.0, std::min(1.0, t));

    double closest_x = x1 + t * (x2 - x1);
    double closest_y = y1 + t * (y2 - y1);

    double dist = calculateDistance(cav_x_in, cav_y_in, closest_x, closest_y);

    if (dist <= detection_radius) {
        cout << "[ZONE3_2_DETECTION] CAV in CAV Zone3_2!" << std::endl;
        return true;
    }

    return false;
}

bool is_cav_in_zone4(double cav_x_in, double cav_y_in) {
    const double detection_radius = 0.2;

    double x1 = cav_zone4_start.x;
    double y1 = cav_zone4_start.y;
    double x2 = cav_zone4_end.x;
    double y2 = cav_zone4_end.y;

    double line_len_sq = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

    if (line_len_sq < 1e-6) {
        return calculateDistance(cav_x_in, cav_y_in, x1, y1) <= detection_radius;
    }

    double t = ((cav_x_in - x1) * (x2 - x1) + (cav_y_in - y1) * (y2 - y1)) / line_len_sq;
    t = std::max(0.0, std::min(1.0, t));

    double closest_x = x1 + t * (x2 - x1);
    double closest_y = y1 + t * (y2 - y1);

    double dist = calculateDistance(cav_x_in, cav_y_in, closest_x, closest_y);

    if (dist <= detection_radius) {
        cout << "[ZONE4_DETECTION] CAV in CAV Zone4!" << std::endl;
        return true;
    }

    return false;
}


bool is_cav_in_zone5(double cav_x_in, double cav_y_in) {
    const double detection_radius = 0.2;

    double x1 = cav_zone5_start.x;
    double y1 = cav_zone5_start.y;
    double x2 = cav_zone5_end.x;
    double y2 = cav_zone5_end.y;

    double line_len_sq = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

    if (line_len_sq < 1e-6) {
        return calculateDistance(cav_x_in, cav_y_in, x1, y1) <= detection_radius;
    }

    double t = ((cav_x_in - x1) * (x2 - x1) + (cav_y_in - y1) * (y2 - y1)) / line_len_sq;
    t = std::max(0.0, std::min(1.0, t));

    double closest_x = x1 + t * (x2 - x1);
    double closest_y = y1 + t * (y2 - y1);

    double dist = calculateDistance(cav_x_in, cav_y_in, closest_x, closest_y);

    if (dist <= detection_radius) {
        cout << "[ZONE5_DETECTION] CAV in CAV Zone5!" << std::endl;
        return true;
    }

    return false;
}

/// HV Zone 6 안에 HV가 있는지 체크 (선분으로부터 거리)
bool is_hv_in_zone6(double hv_x, double hv_y) {
    const double detection_radius = 0.1;

    double x1 = hv_zone6_start.x;
    double y1 = hv_zone6_start.y;
    double x2 = hv_zone6_end.x;
    double y2 = hv_zone6_end.y;

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

/// CAV Zone 안에 CAV가 있는지 체크 (선분으로부터 거리)
bool is_cav_in_zone6(double cav_x_in, double cav_y_in) {
    const double detection_radius = 0.1;

    double x1 = cav_zone6_start.x;
    double y1 = cav_zone6_start.y;
    double x2 = cav_zone6_end.x;
    double y2 = cav_zone6_end.y;

    double line_len_sq = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

    if (line_len_sq < 1e-6) {
        return calculateDistance(cav_x_in, cav_y_in, x1, y1) <= detection_radius;
    }

    double t = ((cav_x_in - x1) * (x2 - x1) + (cav_y_in - y1) * (y2 - y1)) / line_len_sq;
    t = std::max(0.0, std::min(1.0, t));

    double closest_x = x1 + t * (x2 - x1);
    double closest_y = y1 + t * (y2 - y1);

    double dist = calculateDistance(cav_x_in, cav_y_in, closest_x, closest_y);

    if (dist <= detection_radius) {
        cout << "[ZONE6_DETECTION] CAV in CAV Zone6!" << std::endl;
        return true;
    }

    return false;
}


/// Zone 6 충돌 검사: HV Zone에 HV 있고 + CAV Zone에 CAV 있으면 충돌
bool check_zone6_collision(double cav_x_in, double cav_y_in) {
    bool cav_in_zone6 = is_cav_in_zone6(cav_x_in, cav_y_in);
    if (!cav_in_zone6) {
        return false;
    }

    for (const auto& [hv_id, hv_pos] : hv_positions) {
        if (is_hv_in_zone6(hv_pos.first, hv_pos.second)) {
            std::cout << "[ZONE_COLLISION] HV_" << hv_id << " in HV Zone + CAV in CAV Zone!" << std::endl;
            return true;
        }
    }
    return false;
}



// ============================================
/// HV 속도 측정 콜백 (수정됨: 즉시 측정 로직)
// ============================================

/// HV 속도 측정 콜백 (Moving Average 방식)
void MeasureHVVelocity(const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                       int hv_id,
                       std::shared_ptr<rclcpp::Node> node) {

    // (옵션) 경로 데이터가 로드된 HV만 처리하려면 아래 주석 해제
    // if (hv_paths.find(hv_id) == hv_paths.end() || hv_paths[hv_id].empty()) return;

    HVState& state = hv_states[hv_id];
    state.id = hv_id;

    rclcpp::Time current_time = msg->header.stamp;
    double current_x = msg->pose.position.x;
    double current_y = msg->pose.position.y;

    // 1. 첫 메시지 처리
    if (state.is_first_msg) {
        state.prev_x = current_x;
        state.prev_y = current_y;
        state.last_time = current_time;
        state.is_first_msg = false;
        return;
    }

    // 2. 시간 차이 계산
    double dt = (current_time - state.last_time).seconds();
    
    // dt가 너무 작으면 계산 생략 (0.001초 미만 등)
    if (dt < 0.001) return;

    // 3. 이동 거리 및 속도 계산
    double dx = current_x - state.prev_x;
    double dy = current_y - state.prev_y;
    double dist = std::hypot(dx, dy);
    
    double current_vel = dist / dt;

    // 4. 이동 평균 필터 적용
    if (state.vel_buffer.size() >= state.buffer_size) {
        state.vel_buffer.pop_front();
    }
    state.vel_buffer.push_back(current_vel);

    double sum = std::accumulate(state.vel_buffer.begin(), state.vel_buffer.end(), 0.0);
    double avg_vel = sum / state.vel_buffer.size();

    // 5. 노이즈 및 상한선 처리
    if (avg_vel < 0.01) avg_vel = 0.0; // 정지 상태 처리
    if (avg_vel > 2.5) avg_vel = 2.5;  // 비정상적으로 튀는 값 제한

    // 6. 전역 변수 업데이트
    if (hv_id == 20) {
        measured_hv20_vel = avg_vel;
        // 디버깅용 로그 (필요 시 주석 해제)
        // RCLCPP_INFO(node->get_logger(), "[HV20] Vel: %.2f", avg_vel);
    } else if (hv_id == 24) {
        measured_hv24_vel = avg_vel;
        // RCLCPP_INFO(node->get_logger(), "[HV24] Vel: %.2f", avg_vel);
    }

    // 7. 상태 업데이트
    state.prev_x = current_x;
    state.prev_y = current_y;
    state.last_time = current_time;
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
// [NEW] Overlap Detection Functiona
// =========================
// Lane 2와 Lane 3가 현재 위치(cav_x, cav_y) 기준으로 겹쳐있는지 확인하는 함수
// bool check_overlap_lane23(double x, double y) {
//     // 데이터가 없으면 검사 불가
//     if (lane_paths[2].empty() || lane_paths[3].empty()) return false;

//     // 현재 위치에서 각 차선의 가장 가까운 인덱스 계산
//     // (기존 get_lane_start_idx 함수 재사용)
//     int idx2 = get_lane_start_idx(2, x, y);
//     int idx3 = get_lane_start_idx(3, x, y);

//     if (idx2 < 0 || idx3 < 0) return false;

//     double p2_x = lane_paths[2][idx2].x;
//     double p2_y = lane_paths[2][idx2].y;
//     double p3_x = lane_paths[3][idx3].x;
//     double p3_y = lane_paths[3][idx3].y;

//     // 두 차선의 가장 가까운 점 사이의 거리 계산
//     double separation = std::hypot(p2_x - p3_x, p2_y - p3_y);

//     // 거리가 0.2m 미만이면 사실상 같은 경로(중복)로 판단
//     return (separation < 0.2);
// }

// Lane 2와 Lane 3가 겹치는 구간(인덱스 범위)을 미리 계산하는 함수
void init_overlap_region() {
    if (lane_paths[2].empty() || lane_paths[3].empty()) {
        std::cout << "[OVERLAP_INIT] Paths are empty. Skipping." << std::endl;
        return;
    }

    double threshold = 0.2; // 겹침 판단 거리 (m)
    int path3_size = (int)lane_paths[3].size();
    
    int min_idx = -1;
    int max_idx = -1;
    bool in_overlap = false;

    // Lane 3의 모든 점을 순회
    for (int i = 0; i < path3_size; ++i) {
        double p3_x = lane_paths[3][i].x;
        double p3_y = lane_paths[3][i].y;

        // Lane 2에서 가장 가까운 점 찾기 (단순 탐색)
        // (초기화 단계이므로 성능 최적화보다는 정확성 위주로 전체 탐색 or 근처 탐색)
        double min_dist = 1e10;
        for (const auto& p2 : lane_paths[2]) {
            double d = std::hypot(p3_x - p2.x, p3_y - p2.y);
            if (d < min_dist) min_dist = d;
        }

        // 겹침 판정
        if (min_dist <= threshold) {
            if (min_idx == -1) min_idx = i; // 최초 발견 지점
            max_idx = i; // 계속 갱신하여 마지막 지점 저장
            in_overlap = true;
        }
    }

    if (in_overlap && min_idx != -1 && max_idx != -1) {
        overlap_start_idx = min_idx;
        overlap_end_idx = max_idx;
        overlap_detected = true;
        std::cout << "========================================" << std::endl;
        std::cout << "[OVERLAP_INIT] Overlap Region Detected!" << std::endl;
        std::cout << "   Range (Lane 3 Index): " << overlap_start_idx << " ~ " << overlap_end_idx << std::endl;
        std::cout << "========================================" << std::endl;
    } else {
        std::cout << "[OVERLAP_INIT] No significant overlap found." << std::endl;
    }
}

// 현재 Lane 3 위에서의 내 위치(인덱스)가 겹침 구간에 근접했는지 확인
bool check_approaching_overlap(int current_lane3_idx) {
    if (!overlap_detected || overlap_start_idx == -1) return false;

    // 미리 막기 위한 여유 버퍼 (인덱스 개수)
    // 예: 50개 포인트(약 5~10m) 전부터 Lane 3를 막음
    int safety_margin = 30; 

    // 순환 경로(Circular) 고려 없이 단순 선형 비교일 경우:
    // [overlap_start - margin]  ~  [overlap_end] 구간에 있으면 true
    
    // 만약 경로가 순환한다면 인덱스 랩어라운드 처리가 필요하지만, 
    // 여기서는 단순 구간 비교로 구현합니다.
    
    int block_start = overlap_start_idx - safety_margin;
    int block_end = overlap_end_idx + safety_margin;

    // 현재 인덱스가 이 구간 안에 들어오면 true
    if (current_lane3_idx >= block_start && current_lane3_idx <= block_end) {
        return true;
    }
    
    return false;
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
    if (path.empty()) return false; // 예외 처리 추가
    
    int path_size = (int)path.size(); // 전체 경로 크기

    // Check points ahead
    std::vector<int> roi_offsets = {-20, -15, -10, -5, -4, -3, -2, -1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80, 90};
    
    for (int offset : roi_offsets) {
        int check_idx = start_idx + offset;
        
        // [수정] 순환(Circular) 인덱스 처리
        // 범위를 벗어나면 반대편으로 넘김
        if (check_idx >= path_size) {
            check_idx -= path_size; 
        } else if (check_idx < 0) {
            check_idx += path_size;
        }
        
        // 혹시라도 offset이 path_size보다 클 경우를 대비한 안전장치 (Modulo 연산)
        // check_idx = (check_idx % path_size + path_size) % path_size; // 위 if문으로 커버되면 생략 가능

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

int choose_lane(const std::vector<bool>& collision_list, int current_lane, bool in_zone3, bool in_zone4, bool in_zone5) {
    std::vector<int> priority;
    priority.push_back(current_lane);
    bool solid_lane1 = is_cav_in_zone3(cav_x, cav_y);
    bool solid_lane2 = is_cav_in_zone4(cav_x, cav_y);

    if (collision_list[1] && collision_list[2] && collision_list[3]) {
        return current_lane;
    }

    if (in_zone3 && in_zone4 && in_zone5) {
        return current_lane;
    }

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

    // [수정된 부분 시작] ==========================================
    // 기존: bool is_overlap = check_overlap_lane23(cav_x, cav_y);
    
    // 변경: 좌표(x,y)를 Lane 3 기준 인덱스로 변환 후 검사
    int current_idx_on_3 = get_lane_start_idx(3, cav_x, cav_y);
    bool is_overlap = check_approaching_overlap(current_idx_on_3);
    // [수정된 부분 끝] ============================================
    
    if (is_overlap && (new_lane == 3 || new_lane == 2)) {
        RCLCPP_WARN(node->get_logger(), 
            "[OVERLAP] Path 2 & 3 merged. Forcing Lane 3 -> Lane 2 recognition.");
        new_lane = 2; // 강제로 2번 차선으로 인식하게 함
    }

    // 이미 해당 차선이면 리턴
    if (new_lane == current_lane) return;

    // 경로 교체 및 상태 업데이트
    integrate_path_vector = lane_paths[new_lane];
    current_lane = new_lane;
    
    lane_start_idx[new_lane] = get_lane_start_idx(new_lane, cav_x, cav_y);

    RCLCPP_INFO(node->get_logger(), 
                "[LANE_SWITCH] Switched to Lane %d (size: %zu, start_idx: %d)", 
                new_lane, integrate_path_vector.size(), lane_start_idx[new_lane]);
}


bool is_race_over (CavState& cav) {
    // 1. 초기화 안 된 경우 시작점 설정
    if (!cav.is_initialized) {
        cav.id = 1; // 기본 ID 설정 (필요시 수정)
        cav.start_x = cav_x;
        cav.start_y = cav_y;
        cav.start_yaw = cav_yaw;
        std::cout << "[CAV " << cav.id << "] Start Point Set: (" << cav.start_x << ", " << cav.start_y << ")" << std::endl << std::endl;
        cav.is_in_line = true; // 시작하자마자는 라인 안에 있다고 가정
        cav.is_initialized = true;
    }

    double start_dx = cav_x - cav.start_x;
    double start_dy = cav_y - cav.start_y;

    double start_cos = std::cos(cav.start_yaw);
    double start_sin = std::sin(cav.start_yaw);

    // 로컬 좌표로 변환 (start_x, start_y 기준)
    double local_lon = start_cos * start_dx + start_sin * start_dy;
    double local_lat = -start_sin * start_dx + start_cos * start_dy;

    // 결승선 통과 조건 (출발점 기준 앞 0~0.5m, 좌우 0.8m 이내)
    bool line_thickness = (local_lon >= 0.0 && local_lon <= 0.5);
    bool line_width = (std::abs(local_lat) <= 0.8);
    bool on_finish_line = line_thickness && line_width;

    if (on_finish_line) {
        // 이전에 라인 밖에 있었다가 들어온 경우 -> 랩 카운트 증가
        if(!cav.is_in_line) {
            cav.current_lap++;
            cav.is_in_line = true;

            std::cout << "\n>>> [LAP UPDATE] Passed Gate! Lap " << cav.current_lap << " / 5 <<<" << std::endl;
            std::cout << "    (Forward Dist: " << local_lon << "m, Side: " << local_lat << "m)" << std::endl;

            if (cav.current_lap >= 5) {
                cav.is_finished = true;
                std::cout << ">>> [FINISH] 5 LAPS COMPLETED! STOPPING CAR... <<<" << std::endl;
            }
        }
    } else {
        // 라인 밖으로 벗어남 (한 바퀴 도는 중)
        cav.is_in_line = false;
    }

    return cav.is_finished; // [DEBUG] 리턴문 추가됨
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

    int path_size = (int)path.size();

    // [수정] 현재 위치부터 경로 길이만큼 순환하며 탐색
    for (int i = 0; i < path_size; i++) {
        int target_idx = (closest_idx + i) % path_size; // 순환 인덱스
        
        double d = calculateDistance(x, y, path[target_idx].x, path[target_idx].y);
        if (d > lookahead) {
            return target_idx;
        }
    }

    // 못 찾으면 현재 위치 리턴 (혹은 경로의 마지막)
    return closest_idx;
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
        st.speed_mps = 1.3;
    } else {
        st.speed_mps = 2.0;
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
    double max_ld = 0.45;
    double min_ld = 0.15;
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
    auto st2 = std::make_shared<CavState>();

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
    for (int lane = 1; lane <= 3; ++lane) {
        std::string csv_path = std::string("/root/TEAM_AIM/src/global_path/") +
                               "path_mission2_" + std::to_string(cav_id) + "_" + twoDigitId(lane) + ".csv";
        
        if (!loadPathCsv(csv_path, lane_paths[lane])) {
            RCLCPP_ERROR(node->get_logger(), "[PRELOAD] Failed to load lane %d", lane);
            rclcpp::shutdown();
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "[PRELOAD] Lane %d loaded: %zu points", 
                    lane, lane_paths[lane].size());
    }

    // HV 경로 로드 (추가!)
    RCLCPP_INFO(node->get_logger(), "[PRELOAD] Loading HV paths for velocity measurement...");

    // HV20 (Lane 3) 경로
    std::string hv20_path = "/root/TEAM_AIM/src/global_path/path_mission2_1_20.csv";
    if (loadPathCsv(hv20_path, hv_paths[20])) {
        RCLCPP_INFO(node->get_logger(), "[PRELOAD] HV20 path loaded: %zu points", hv_paths[20].size());
    } else {
        RCLCPP_WARN(node->get_logger(), "[PRELOAD] Failed to load HV20 path");
    }
    
    // HV24 (Lane 2) 경로
    std::string hv24_path = "/root/TEAM_AIM/src/global_path/path_mission2_1_24.csv";
    if (loadPathCsv(hv24_path, hv_paths[24])) {
        RCLCPP_INFO(node->get_logger(), "[PRELOAD] HV24 path loaded: %zu points", hv_paths[24].size());
    } else {
        RCLCPP_WARN(node->get_logger(), "[PRELOAD] Failed to load HV24 path");
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
        red_flag_topic, 1,
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

    init_overlap_region();
    // Set initial path
    current_lane = 2;
    integrate_path_vector = lane_paths[current_lane];
    lane_start_idx[current_lane] = 0;

    // ---- CAV pose subscription and control loop
    static int pose_callback_count = 0;
    auto last_log_time = std::chrono::steady_clock::now();
    const int LOG_INTERVAL_MS = 2000; // 로그 주기 단축

    auto pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic, rclcpp::SensorDataQoS(),
        [node, st, st2, accel_pub, cav_id, &last_log_time](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            
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
            zone_cav_flag = is_cav_in_zone(cav_x, cav_y);

            // ===== 2-2. ZONE 2 충돌 검사 =====
            zone2_collision_flag = check_zone2_collision();

            // ===== 2-3. ZONE 3 & 4 & 5 충돌 검사 =====
            zone3_collision_flag = is_cav_in_zone3(cav_x, cav_y);
            zone3_1_collision_flag = is_cav_in_zone3_1(cav_x, cav_y);
            zone3_2_collision_flag = is_cav_in_zone3_2(cav_x, cav_y);
            zone4_collision_flag = is_cav_in_zone4(cav_x, cav_y);
            zone5_collision_flag = is_cav_in_zone5(cav_x, cav_y);
            zone6_collision_flag = check_zone6_collision(cav_x, cav_y);

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
            // zone2 충돌 감지 시 1, 2차선 막힘 처리 (안정성 강화)
            if (zone2_collision_flag) {         
                lane_collision[2] = true;
                lane_collision[1] = true;
            }
            // zon3_2 주행 시 1차선 막힘 처리(1차선으로 가면 장애물이 많기에 3차선 주행이 rap time 향상에 유리)
            if (current_lane == 2 && zone3_2_collision_flag) {         
                lane_collision[1] = true;
                lane_collision[2] = true; // 1,2차선 모두 막음으로 3차선 유지 유도
            }
            // zone3/zone5 는 실선이므로 차선변경 불가능하게 막음 (실선 차선 변경 불가능)
            else if (current_lane == 2 && zone5_collision_flag || zone3_collision_flag) {
                lane_collision[3] = true;
            }
            else if (current_lane == 3 && zone6_collision_flag) {
                lane_collision[3] = true;
                lane_collision[2] = true;
                lane_collision[1] = true;
            }
            // zone4 는 실선이므로 차선변경 불가능하게 막음 (실선 차선 변경 불가능)
            else if (current_lane == 3 && zone3_1_collision_flag || zone4_collision_flag) {
                lane_collision[2] = true;
                lane_collision[1] = true;
            }
            else if (zone_collision_flag) {
                lane_collision[3] = true;
            }

            // =================================================================================
            // [MODIFIED] 미리 계산된 인덱스 기반 Overlap 감지
            // =================================================================================
            // 현재 내 위치가 Lane 3 경로 상에서 어디쯤(인덱스)인지 확인
            int my_idx_on_lane3 = get_lane_start_idx(3, cav_x, cav_y);
            
            // 겹침 구간(또는 그 직전)에 진입했는지 확인
            bool is_overlap_zone = check_approaching_overlap(my_idx_on_lane3);
            
            if (is_overlap_zone) {
                // 1) Lane 3는 막힌 것으로 처리 (미리 차단)
                lane_collision[3] = true;
                
                // 2) 만약 현재 Lane 3를 달리고 있다면, 강제로 Lane 2로 변경
                if (current_lane == 3) {
                    RCLCPP_WARN(node->get_logger(), "[OVERLAP] Approaching merge zone! Forcing Lane 3 -> Lane 2.");
                    change_csv_state(cav_id, 2, node);
                }
            }

            // ===== 5. Check if should STOP =====
            bool stop_flag = should_stop(lane_collision, current_lane);
            
            // ===== 6. Choose passable lane =====
            // lane_collision[3]이 true이므로, current=2일 때 Lane 1을 선택하게 됨
            int next_lane = choose_lane(lane_collision, current_lane, zone3_collision_flag, zone4_collision_flag, zone5_collision_flag);

            // ===== 7. Lane switch if needed =====
            // Zone1/Zone2/Zone3/Zone4/Zone5 충돌 없고, stop_flag 아니면 차선 변경
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
            if (!zone_collision_flag && !zone2_collision_flag && !stop_flag && next_lane != current_lane) { 
                 change_csv_state(cav_id, next_lane, node);
            }

            // ===== 9. Race Over Logic =======
            bool race_over = is_race_over(*st2);

            // 10. Log Lane Status (수정됨: 상태 확인용)
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count();
            
            if (elapsed >= LOG_INTERVAL_MS) {
                std::cout << "\n=== [Lane Status] (CAV" << cav_id << ") ===" << std::endl;
                std::cout << "CAV Pos: (" << std::fixed << std::setprecision(2) << cav_x << ", " << cav_y << ")" << std::endl;
                
                if (is_overlap_zone) { // 변수명을 is_overlap_zone으로 변경
                    std::cout << ">>> OVERLAP DETECTED! Forcing Lane 3 -> Lane 2 logic <<<" << std::endl;
                }

                for (int lane = 1; lane <= 3; ++lane) {
                    std::string status = lane_collision[lane] ? "X OCCUPIED" : "O FREE";
                    if (is_overlap_zone && lane == 3) status += " (Overlap Force)"; // 변수명 변경
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

            // ===== 11. Publish Control (수정: 우선순위 재정렬) =====
            geometry_msgs::msg::Accel cmd;

            // [우선순위 1] Race Over (경기 종료 시 무조건 정지)
            if (race_over) {
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
            }
            // [우선순위 2] Zone 1 또는 Zone 2 충돌 감지 시 '무조건' 정지 (Stop Flag보다 먼저 검사해야 함!)
            else if (zone2_collision_flag) {
                // 로그를 띄워서 정지 원인 확인 (과도한 로그 방지를 위해 Throttle 사용)
                RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000, 
                    "[EMERGENCY] Zone Collision! Stopping. (Zone1: %d, Zone2: %d)", 
                    zone_collision_flag, zone2_collision_flag);
                
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
            }
            // [우선순위 3] 일반 Stop Flag (앞차 거리 유지 등) -> 감속하거나 정지
            else if (stop_flag) {                       
                // 각 차선별 감속 로직   
                if (current_lane == 3 && zone2_collision_flag) {
                    cmd.linear.x = 0.0; // 3차선은 완전 정지
                    cmd.angular.z = 0.0;
                }
                else if (current_lane == 2) {
                    cmd.linear.x = measured_hv24_vel; // HV24 속도 추종 (감속)
                    cmd.angular.z = wz;
                }
                else if (current_lane == 3) {
                    cmd.linear.x = measured_hv20_vel; // HV20 속도 추종 (감속)
                    cmd.angular.z = wz;
                }
                else if (current_lane == 1) {
                    cmd.linear.x = 0.0; // 1차선은 완전 정지
                    cmd.angular.z = 0.0;
                }
                else {
                    cmd.linear.x = 0.0; 
                    cmd.angular.z = 0.0;
                }
            }
            // [우선순위 4] 그 외 일반 주행
            else {
                cmd.linear.x = st->speed_mps;
                cmd.angular.z = wz;
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