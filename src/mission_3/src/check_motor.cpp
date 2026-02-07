#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp> 

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
#include <chrono>
#include <thread>

using namespace std;

// =========================
// Path data
// =========================
struct integrate_path_struct {
  double x;
  double y;
};

static std::vector<integrate_path_struct> integrate_path_vector;

// =========================
// Localization globals
// =========================
inline double x_m = 0.0;
inline double y_m = 0.0;
inline double z_m = 0.0;

inline double x_q = 0.0;
inline double y_q = 0.0;
inline double z_q = 0.0;
inline double w_q = 0.0;

inline void get_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                     double &x_m, double &y_m, double &z_m,
                     double &x_q, double &y_q, double &z_q, double &w_q)
{
  x_m = msg->pose.position.x;
  y_m = msg->pose.position.y;
  z_m = msg->pose.position.z;

  x_q = msg->pose.orientation.x;
  y_q = msg->pose.orientation.y;
  z_q = msg->pose.orientation.z;
  w_q = msg->pose.orientation.w;
}

// =========================
// Controller state
// =========================
struct ControllerState
{
  double speed_mps{0.5};
  double lookahead_m{0.3};
  double max_yaw_rate{2.0}; // ** 5.5 (noise error) -> 2.5 (tuning) ** -> 2.0

  double prev_x{0.0};
  double prev_y{0.0};
  double prev_yaw{0.0};
  bool has_prev{false};

  int pos_count{0};
  int red_flag{0}; 
  int yellow_flag{0};  // Yellow flag for speed control
  bool tower_mode{false};
  double target_linear_x{0.0};
};

// =========================
// CAV Structures
// =========================
struct CavState {
    int id;                 
    double start_x;         
    double start_y;         
    int current_lap;        
    bool is_initialized;    
    bool is_in_zone;        
    bool finished;
    rclcpp::Time lap_start_time;  // Lap start time for lap timing
};

// =========================
// Helper Functions
// =========================
static double yawFromQuat(const geometry_msgs::msg::Quaternion& qmsg, double fallback_yaw)
{
  const double norm2 = qmsg.x*qmsg.x + qmsg.y*qmsg.y + qmsg.z*qmsg.z + qmsg.w*qmsg.w;
  if (norm2 <= 1e-6) return fallback_yaw;

  tf2::Quaternion q(qmsg.x, qmsg.y, qmsg.z, qmsg.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

static int readCavIdFromEnvOrDefault(int default_id)
{
  const char* s = std::getenv("CAV_ID");
  if (!s || std::string(s).empty()) return default_id;
  try { return std::stoi(s); } catch (...) { return default_id; }
}

static std::string twoDigitId(int id)
{
  if (id < 0) id = 0;
  if (id > 99) id = 99;

  std::ostringstream oss;
  oss << std::setw(2) << std::setfill('0') << id;
  return oss.str();
}

static bool loadPathCsv(const std::string& csv_path, std::vector<integrate_path_struct>& out)
{
  std::ifstream file(csv_path);
  if (!file.is_open()) return false;

  out.clear();
  std::string line;
  if (std::getline(file, line)) {
    std::stringstream ss(line);
    double x, y; char comma;
    if (ss >> x >> comma >> y) {
      out.push_back({x, y});
    }
  }

  while (std::getline(file, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    double x, y; char comma;
    if (!(ss >> x)) continue;
    if (!(ss >> comma)) continue;
    if (!(ss >> y)) continue;
    if (std::isnan(x) || std::isnan(y)) continue;
    out.push_back({x, y});
  }

  return out.size() >= 2;
}

// =========================
// Waypoint search
// =========================
inline int closest_index = 0;

void GetLd(ControllerState& st) {
  double gain_ld = 0.6; // 0.4 -> 0.6 ** tuning **
  double max_ld  = 0.355;
  double min_ld  = 0.1; // 0.15 -> 0.1
  double velocity = st.speed_mps; 
  double ld = gain_ld * velocity;
  st.lookahead_m = max(min_ld, std::min(max_ld, ld));
}

static int findClosestPoint(const std::vector<integrate_path_struct>& path, double x_m, double y_m)
{
  if (path.empty()) return -1;
  double min_d = -1.0; int best = 0;

  for (size_t i = 0; i < path.size(); i++) {
    const double dx = path[i].x - x_m;
    const double dy = path[i].y - y_m;
    const double d  = std::hypot(dx, dy);

    if (min_d < 0.0 || d < min_d) {
      min_d = d;
      best = static_cast<int>(i);
    }
  }
  closest_index = best;
  return best;
}

// int findWaypoint (const vector<integrate_path_struct>& integrate_path_vector, double x_m, double y_m, const double L_d) {
//     if (integrate_path_vector.empty()) { cout <<"Path Empty" << endl; return -1; }
//     int closest_idx = findClosestPoint(integrate_path_vector, x_m, y_m);
//     for (int i = closest_idx; i < (int)integrate_path_vector.size(); ++i) {
//         double d = hypot(integrate_path_vector[i].x - x_m, integrate_path_vector[i].y - y_m);
//         if (d > L_d) { return i; }
//     }
//     return integrate_path_vector.size() - 1;
// }
int findWaypoint (const vector<integrate_path_struct>& integrate_path_vector, double x_m, double y_m, const double L_d) {
    if (integrate_path_vector.empty()) { cout <<"Path Empty" << endl; return -1; }
    
    int closest_idx = findClosestPoint(integrate_path_vector, x_m, y_m);
    int path_size = (int)integrate_path_vector.size();

    // Loop를 통해 순환 탐색 (최대 전체 경로 길이만큼 확인)
    for (int i = 0; i < path_size; ++i) {
        // [핵심 변경] 모듈러 연산(%)을 사용하여 인덱스가 끝을 넘어가면 0으로 돌아오게 함
        int current_idx = (closest_idx + i) % path_size;

        double d = hypot(integrate_path_vector[current_idx].x - x_m, integrate_path_vector[current_idx].y - y_m);
        if (d > L_d) { 
            return current_idx; 
        }
    }
    
    // 혹시라도 못 찾으면 현재 위치 바로 다음 점 반환 (순환 고려)
    return (closest_idx + 1) % path_size;
}

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

bool isCorner(const vector<integrate_path_struct>& integrate_path_vector, double /*L_d*/, int closest_idx) {
    // int future_offset = 20; // 20 -> 60
    int future_offset = 60;
    int path_size = (int)integrate_path_vector.size();
    if (closest_idx + future_offset + 1 >= path_size) { return false; }

    double cur_ang = atan2(integrate_path_vector[closest_idx + 1].y - integrate_path_vector[closest_idx].y, 
                           integrate_path_vector[closest_idx + 1].x - integrate_path_vector[closest_idx].x);
    int idx_future_start = closest_idx + future_offset;
    double fut_ang = atan2(integrate_path_vector[idx_future_start + 1].y - integrate_path_vector[idx_future_start].y, 
                           integrate_path_vector[idx_future_start + 1].x - integrate_path_vector[idx_future_start].x);
    
    double diff = fabs(normalizeAngle(fut_ang - cur_ang));
    if (diff > M_PI / 2) { diff = M_PI - diff; }
    
    double threshold_deg = 10.0; 
    double threshold_rad = threshold_deg * (M_PI / 180.0);
    return (diff > threshold_rad);
}


// Global mission completion flag
static bool mission_completed = false;
static int stop_publish_count = 0;

// =========================
// main
// =========================
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("check_motor");

  // Get CAV_ID from environment variable
  int cav_id_default = readCavIdFromEnvOrDefault(1);
  node->declare_parameter<int>("cav_id", cav_id_default);
  const int actual_cav_id = node->get_parameter("cav_id").as_int();

  const std::string my_id_str = twoDigitId(actual_cav_id);
  const std::string cmd_vel_topic = "/CAV_" + my_id_str + "/cmd_vel_"; // motor_topic name tuned

  RCLCPP_INFO(node->get_logger(), "===== Motor Check Started =====");
  RCLCPP_INFO(node->get_logger(), "CAV_ID: %d", actual_cav_id);
  RCLCPP_INFO(node->get_logger(), "Publishing to topic: %s", cmd_vel_topic.c_str());

  // Create publisher
  auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, rclcpp::QoS(10));

  // Wait a bit for publisher to be ready
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  RCLCPP_INFO(node->get_logger(), "===== Starting Motor Test =====");
  RCLCPP_INFO(node->get_logger(), "Phase 1: 3 seconds at 0.5 m/s");

  // Phase 1: 3 seconds at 0.5 m/s
  geometry_msgs::msg::Twist msg;
  msg.linear.x = 0.5;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;

  auto start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(3)) {
    cmd_vel_pub->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  RCLCPP_INFO(node->get_logger(), "Phase 1 Complete (0.5 m/s)");

  // Phase 2: 3 seconds at 1.0 m/s
  RCLCPP_INFO(node->get_logger(), "Phase 2: 3 seconds at 1.0 m/s");
  msg.linear.x = 1.0;
  start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(3)) {
    cmd_vel_pub->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  RCLCPP_INFO(node->get_logger(), "Phase 2 Complete (1.0 m/s)");

  // Phase 3: 3 seconds at 1.5 m/s
  RCLCPP_INFO(node->get_logger(), "Phase 3: 3 seconds at 1.5 m/s");
  msg.linear.x = 1.5;
  msg.angular.z = 3.0;
  start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(3)) {
    cmd_vel_pub->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  RCLCPP_INFO(node->get_logger(), "Phase 3 Complete (1.5 m/s)");

  // Stop
  RCLCPP_INFO(node->get_logger(), "===== Stopping Motor =====");
  msg.linear.x = 0.0;
  for (int i = 0; i < 10; ++i) {
    cmd_vel_pub->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  RCLCPP_INFO(node->get_logger(), "===== Motor Check Complete =====");
  
  rclcpp::shutdown();
  return 0;
}