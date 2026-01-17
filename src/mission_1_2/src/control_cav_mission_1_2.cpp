#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp> // [수정] Float64 사용

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
  double max_yaw_rate{4.0}; // [핵심 수정] 1.5 -> 4.0 (고속 회전 허용)

  double prev_x{0.0};
  double prev_y{0.0};
  double prev_yaw{0.0};
  bool has_prev{false};

  int pos_count{0};
  int red_flag{0}; 

  // 타워 모드 변수
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
    std::stringstream ss(line); double x, y; char comma;
    if (!(ss >> x >> comma >> y)) { } else { out.push_back({x, y}); }
  }

  while (std::getline(file, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line); double x, y; char comma;
    if (!(ss >> x)) continue; if (!(ss >> comma)) continue; if (!(ss >> y)) continue;
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
  double gain_ld = 0.4; 
  // [핵심 수정] 고속에서도 너무 멀리 보지 않도록 max_ld를 타이트하게 잡음
  double max_ld = 0.355; 
  double min_ld = 0.15; // 최소값도 너무 작지 않게
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

int findWaypoint (const vector<integrate_path_struct>& integrate_path_vector, double x_m, double y_m, const double L_d) {
    if (integrate_path_vector.empty()) { cout <<"Path Empty" << endl; return -1; }
    int closest_idx = findClosestPoint(integrate_path_vector, x_m, y_m);
    for (int i = closest_idx; i < (int)integrate_path_vector.size(); ++i) {
        double d = hypot(integrate_path_vector[i].x - x_m, integrate_path_vector[i].y - y_m);
        if (d > L_d) { return i; }
    }
    return integrate_path_vector.size() - 1;
}

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

bool isCorner(const vector<integrate_path_struct>& integrate_path_vector, double /*L_d*/, int closest_idx) {
    int future_offset = 20; 
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

void planVelocity(ControllerState& st, bool isCorner) {
    if (!isCorner) { st.speed_mps = 2.0; } else { st.speed_mps = 1.5; }
}

bool CheckAllFinished(const std::vector<CavState>& cav_list) {
    int finished_count = 0;
    for (int i = 0; i < (int)cav_list.size(); ++i) {
        if (cav_list[i].finished) finished_count++;
    }
    if (finished_count == (int)cav_list.size() - 1) return true;
    return false;
}

void PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int cav_id, std::vector<CavState>& states) {
    CavState& current_cav = states[cav_id];
    double current_x = msg->pose.position.x;
    double current_y = msg->pose.position.y;

    if (!current_cav.is_initialized) {
        current_cav.start_x = current_x;
        current_cav.start_y = current_y;
        current_cav.is_initialized = true;
        current_cav.is_in_zone = true; 
        std::cout << "[CAV " << cav_id << "] Start Point Set: (" << current_x << ", " << current_y << ")" << std::endl << std::endl;
        return;
    }

    if (current_cav.finished) return;

    double dist_to_start = std::hypot(current_x - current_cav.start_x, current_y - current_cav.start_y);

    if (dist_to_start < 0.1) {
        if (!current_cav.is_in_zone) {
            current_cav.current_lap += 1;
            current_cav.is_in_zone = true; 
            std::cout << "[CAV " << cav_id << "] Lap Increased! Current Lap: " << current_cav.current_lap << std::endl << std::endl;
            if (current_cav.current_lap >= 5) {
                current_cav.finished = true;
                std::cout << "[CAV " << cav_id << "] FINISHED 5 LAPS!" << std::endl << std::endl;
            }
        }
    } else {
        if (current_cav.is_in_zone) current_cav.is_in_zone = false; 
    }
}

// =========================
// main
// =========================
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("control");
  auto st   = std::make_shared<ControllerState>();

  int total_vehicle_count = 2;
  std::vector<CavState> cav_list(total_vehicle_count + 1); 
  for(int i=1; i<(int)cav_list.size(); ++i) cav_list[i] = {i, 0.0, 0.0, 0, false, false, false};

  int cav_id_default = readCavIdFromEnvOrDefault(1);
  node->declare_parameter<int>("cav_id", cav_id_default);
  const int cav_id = node->get_parameter("cav_id").as_int();

  if(cav_id >= (int)cav_list.size()) {
      RCLCPP_ERROR(node->get_logger(), "CAV_ID(%d) Error", cav_id);
      return -1;
  }

  const std::string my_id_str = twoDigitId(cav_id);
  const std::string accel_topic = "/CAV_" + my_id_str + "_accel"; 
  const std::string red_flag_topic = "/CAV_" + my_id_str + "_RED_FLAG";
  const std::string target_vel_topic = "/CAV_" + my_id_str + "_target_vel";

  RCLCPP_INFO(node->get_logger(), "My CAV_ID=%d, Waiting for %d vehicles...", cav_id, total_vehicle_count);

  node->declare_parameter<double>("speed_mps", 0.5);
  node->declare_parameter<double>("lookahead_m", 0.4);
  node->declare_parameter<double>("max_yaw_rate", 4.0); // [핵심] 고속 주행을 위해 Yaw Rate 제한 대폭 해제
  node->declare_parameter<std::string>("path_csv", "/root/TEAM_AIM/src/global_path/path_mission1_01.csv");

  st->speed_mps    = node->get_parameter("speed_mps").as_double();
  st->lookahead_m  = node->get_parameter("lookahead_m").as_double();
  st->max_yaw_rate = node->get_parameter("max_yaw_rate").as_double();

  std::string path_with_id_csv;
  if (my_id_str == "01") {
    path_with_id_csv = "/root/TEAM_AIM/src/global_path/path_mission1_01.csv";
  } else if (my_id_str == "02") {
    path_with_id_csv = "/root/TEAM_AIM/src/global_path/path_mission1_02.csv";
  } else {
    path_with_id_csv = "/root/TEAM_AIM/src/global_path/path_mission1_01.csv";
  }
  if (!loadPathCsv(path_with_id_csv, integrate_path_vector)) {
    RCLCPP_FATAL(node->get_logger(), "Failed to load path csv: %s", path_with_id_csv.c_str());
    rclcpp::shutdown(); return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Loaded path: %zu waypoints from %s", integrate_path_vector.size(), path_with_id_csv.c_str());
 
  auto accel_pub = node->create_publisher<geometry_msgs::msg::Accel>(accel_topic, rclcpp::SensorDataQoS());

  auto flag_sub = node->create_subscription<std_msgs::msg::Int32>(
      red_flag_topic, 10, [node, st](const std_msgs::msg::Int32::SharedPtr msg) { st->red_flag = msg->data; });

  auto vel_sub = node->create_subscription<std_msgs::msg::Float64>(
      target_vel_topic, 10, 
      [st](const std_msgs::msg::Float64::SharedPtr msg) {
          if (msg->data < 0.0) {
              st->tower_mode = false;
          } else {
              st->tower_mode = true;
              st->target_linear_x = msg->data;
          }
      }
  );

  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subs;

  for (int target_id = 1; target_id < total_vehicle_count + 1; ++target_id) {
      std::string target_topic = "/CAV_" + twoDigitId(target_id);
      auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
          target_topic, rclcpp::SensorDataQoS(),
          [node, st, accel_pub, &cav_list, cav_id, target_id](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
          {
            PoseCallback(msg, target_id, cav_list);

            if (target_id == cav_id) {
                if (CheckAllFinished(cav_list)) {
                    geometry_msgs::msg::Accel stop_cmd;
                    stop_cmd.linear.x = 0.0; stop_cmd.linear.y = 0.0; stop_cmd.linear.z = 0.0;
                    stop_cmd.angular.x = 0.0; stop_cmd.angular.y = 0.0; stop_cmd.angular.z = 0.0;
                    accel_pub->publish(stop_cmd);
                    static int stop_log_cnt = 0;
                    if (stop_log_cnt++ % 50 == 0) std::cout << ">>> [ALL FINISHED] SYNC STOP! <<<" << std::endl;
                    return;
                }

                get_pose(msg, x_m, y_m, z_m, x_q, y_q, z_q, w_q);
                double yaw = yawFromQuat(msg->pose.orientation, st->prev_yaw);
                if (st->has_prev && std::hypot(x_m - st->prev_x, y_m - st->prev_y) > 1e-4) {
                    yaw = std::atan2(y_m - st->prev_y, x_m - st->prev_x);
                }
                st->prev_x = x_m; st->prev_y = y_m; st->prev_yaw = yaw; st->has_prev = true;
                
                // [Pure Pursuit Logic]
                // 1. Closest Point & Velocity
                int closest_idx = findClosestPoint(integrate_path_vector, x_m, y_m);
                bool corner_detected = isCorner(integrate_path_vector, st->lookahead_m, closest_idx);

                double current_target_speed;
                if (st->tower_mode) {
                    current_target_speed = st->target_linear_x;
                } else {
                    planVelocity(*st, corner_detected);
                    current_target_speed = st->speed_mps;
                }
                
                // [중요] 타워 모드일 때 속도가 빨라지면 Ld가 너무 커지는 것을 막기 위해
                // GetLd 내부에서 max_ld를 0.35로 꽉 잡고 있습니다.
                st->speed_mps = current_target_speed; 
                GetLd(*st); 

                // 2. Pure Pursuit Geometry
                int target_path_idx = findWaypoint(integrate_path_vector, x_m, y_m, st->lookahead_m);

                const double dx = integrate_path_vector[target_path_idx].x - x_m;
                const double dy = integrate_path_vector[target_path_idx].y - y_m;

                const double cy = std::cos(yaw);
                const double sy = std::sin(yaw);
                const double x_v =  cy * dx + sy * dy;
                const double y_v = -sy * dx + cy * dy;
                const double pp_dist = std::max(1e-3, std::hypot(x_v, y_v));
                const double kappa = (2.0 * y_v) / (pp_dist * pp_dist);
                
                double wz = current_target_speed * kappa;
                wz = std::clamp(wz, -st->max_yaw_rate, st->max_yaw_rate);


                // Command Publish
                geometry_msgs::msg::Accel cmd;
                if (st->red_flag == 1) { 
                    cmd.linear.x  = 0.0;
                    cmd.angular.z = 0.0;
                } else {
                    cmd.linear.x  = current_target_speed;
                    cmd.angular.z = wz;
                }
                cmd.linear.y = 0.0; cmd.linear.z = 0.0;
                cmd.angular.x = 0.0; cmd.angular.y = 0.0;
                
                accel_pub->publish(cmd);

                
            }
          }
      );
      pose_subs.push_back(sub); 
      RCLCPP_INFO(node->get_logger(), "Subscribed to %s", target_topic.c_str());
  }

  (void)flag_sub; (void)vel_sub;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}