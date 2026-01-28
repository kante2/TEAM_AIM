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

void planVelocity(ControllerState& st, bool isCorner) {
    if (!isCorner) {st.speed_mps = 1.5; } else { st.speed_mps = 0.8; } // good (1.5 / 1.0) -> (1.5 / 0.8)
}

bool CheckAllFinished(const std::vector<CavState>& cav_list, int vehicle_count) {
    int finished_count = 0;
    for (int i = 1; i <= vehicle_count; ++i) {
        if (cav_list[i].finished) finished_count++;
    }
    if (finished_count == vehicle_count) return true;
    return false;
}

// Global mission completion flag
static bool mission_completed = false;
static int stop_publish_count = 0;

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
            std::cout << "═══════════════════════════════════════════════════════════════" << std::endl;
            std::cout << "🏁 [CAV " << cav_id << "] Lap: " << current_cav.current_lap << "/5 (";
            std::cout << (current_cav.current_lap * 100 / 5) << "% Complete)" << std::endl;
            std::cout << "   Pos: (" << std::fixed << std::setprecision(2) << current_x << ", " << current_y << ")" << std::endl;
            std::cout << "═══════════════════════════════════════════════════════════════" << std::endl << std::endl;
            if (current_cav.current_lap >= 5) {
                current_cav.finished = true;
                std::cout << "🎉 [CAV " << cav_id << "] ✓ FINISHED 5 LAPS! ✓" << std::endl << std::endl;
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

  int total_vehicle_count = 32;
  std::vector<CavState> cav_list(total_vehicle_count + 1); 
  for(int i=1; i<(int)cav_list.size(); ++i) cav_list[i] = {i, 0.0, 0.0, 0, false, false, false};
  
  // 실제 참여 차량 수
  int actual_vehicle_count = 0;

  int cav_id_default = readCavIdFromEnvOrDefault(1);
  node->declare_parameter<int>("cav_id", cav_id_default);
  const int actual_cav_id = node->get_parameter("cav_id").as_int();

  if(actual_cav_id >= (int)cav_list.size()) {
      RCLCPP_ERROR(node->get_logger(), "CAV_ID(%d) Error", actual_cav_id);
      return -1;
  }

  // Get CAV_IDS from environment variable to determine cav_index (1-4)
  const char* cav_ids_env = std::getenv("CAV_IDS");
  int cav_index = 1;  // Default to 1
  if (cav_ids_env != nullptr && strlen(cav_ids_env) > 0) {
      std::string cav_ids_str(cav_ids_env);
      std::stringstream ss(cav_ids_str);
      std::string token;
      int idx = 1;
      while (std::getline(ss, token, ',')) {
          token.erase(0, token.find_first_not_of(" \t"));
          token.erase(token.find_last_not_of(" \t") + 1);
          if (!token.empty()) {
              actual_vehicle_count++;  // 실제 참여 차량 수 증가
              int id = std::stoi(token);
              if (id == actual_cav_id) {
                  cav_index = idx;
                  break;
              }
              idx++;
          }
      }
  }

  const std::string my_id_str = twoDigitId(actual_cav_id);  // Use actual_cav_id for all topics
  const std::string accel_topic = "/CAV_" + my_id_str + "_accel"; 
  const std::string red_flag_topic = "/CAV_" + my_id_str + "_RED_FLAG";
  const std::string target_vel_topic = "/CAV_" + my_id_str + "_target_vel";
  const std::string cmd_vel_topic = "/CAV_" + my_id_str + "/cmd_vel";

  RCLCPP_INFO(node->get_logger(), "My Actual_CAV_ID=%d, Mapped_Index=%d, Actual_Vehicle_Count=%d", actual_cav_id, cav_index, actual_vehicle_count);

  node->declare_parameter<double>("speed_mps", 0.5);
  node->declare_parameter<double>("lookahead_m", 0.4);
  node->declare_parameter<double>("max_yaw_rate", 5.5); // [핵심] 고속 주행을 위해 Yaw Rate 제한 대폭 해제 // ** 5.5 -> 2.5 ?**
  node->declare_parameter<std::string>("path_csv", "/root/TEAM_AIM/src/global_path/path.csv");

  st->speed_mps    = node->get_parameter("speed_mps").as_double();
  st->lookahead_m  = node->get_parameter("lookahead_m").as_double();
  st->max_yaw_rate = node->get_parameter("max_yaw_rate").as_double();

  const std::string path_with_id_csv = std::string("/root/TEAM_AIM/src/global_path/") + "path_mission3_" + std::string(2 - std::to_string(cav_index).length(), '0') + std::to_string(cav_index) + ".csv";
  if (!loadPathCsv(path_with_id_csv, integrate_path_vector)) {
    RCLCPP_FATAL(node->get_logger(), "Failed to load path csv: %s", path_with_id_csv.c_str());
    rclcpp::shutdown(); return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Loaded path: %zu waypoints", integrate_path_vector.size());
 
  auto accel_pub = node->create_publisher<geometry_msgs::msg::Accel>(accel_topic, rclcpp::SensorDataQoS());
  auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, rclcpp::QoS(10));

  auto flag_sub = node->create_subscription<std_msgs::msg::Int32>(
      red_flag_topic, 50, [node, st](const std_msgs::msg::Int32::SharedPtr msg) { st->red_flag = msg->data; });

  auto vel_sub = node->create_subscription<std_msgs::msg::Float64>(
      target_vel_topic, 50, 
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

  // Only subscribe to my own CAV_ID (use actual_cav_id for pose topic)
  int target_id = cav_index;  // Use cav_index for CSV mapping: 1, 2, 3, 4
  {
      std::string target_topic = "/CAV_" + twoDigitId(actual_cav_id);  // /CAV_32, /CAV_02, /CAV_03, /CAV_04
      auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
          target_topic, rclcpp::SensorDataQoS(),
          [node, st, accel_pub, cmd_vel_pub, &cav_list, actual_cav_id, target_id, actual_vehicle_count](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
          {
            PoseCallback(msg, target_id, cav_list);

            // Check mission completion and set global flag
            if (CheckAllFinished(cav_list, actual_vehicle_count) && !mission_completed) {
                mission_completed = true;
                std::cout << ">>> [ALL " << actual_vehicle_count << " VEHICLES FINISHED] Starting continuous STOP publish! <<<" << std::endl;
            }

            // Always execute control for my own CAV
            {
                // 미션 완료 후 계속 정지 명령 발행 (실제 로봇이 멈추도록)
                if (mission_completed) {
                    geometry_msgs::msg::Accel stop_cmd;
                    stop_cmd.linear.x = 0.0; stop_cmd.linear.y = 0.0; stop_cmd.linear.z = 0.0;
                    stop_cmd.angular.x = 0.0; stop_cmd.angular.y = 0.0; stop_cmd.angular.z = 0.0;
                    accel_pub->publish(stop_cmd);
                    
                    geometry_msgs::msg::Twist stop_twist;
                    stop_twist.linear.x = 0.0; stop_twist.linear.y = 0.0; stop_twist.linear.z = 0.0;
                    stop_twist.angular.x = 0.0; stop_twist.angular.y = 0.0; stop_twist.angular.z = 0.0;
                    cmd_vel_pub->publish(stop_twist);
                    
                    if (stop_publish_count++ % 100 == 0) {
                        std::cout << "[STOP] Continuous STOP command published (" << stop_publish_count << ")" << std::endl;
                    }
                    return;
                }

                get_pose(msg, x_m, y_m, z_m, x_q, y_q, z_q, w_q);
                double yaw = yawFromQuat(msg->pose.orientation, st->prev_yaw);
                if (st->has_prev && std::hypot(x_m - st->prev_x, y_m - st->prev_y) > 1e-4) {
                    yaw = std::atan2(y_m - st->prev_y, x_m - st->prev_x);
                }
                st->prev_x = x_m; st->prev_y = y_m; st->prev_yaw = yaw; st->has_prev = true;
                
                // 퓨어퍼슈트 로직
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
                geometry_msgs::msg::Twist twist_cmd;
                if (st->red_flag == 1) { 
                    cmd.linear.x  = 0.0;
                    cmd.angular.z = 0.0;
                    twist_cmd.linear.x = -0.005; // ** -0.01 로 해야지 실제 모터가 정지를 함
                    twist_cmd.angular.z = 0.0;
                } else {
                    cmd.linear.x  = current_target_speed;
                    cmd.angular.z = wz;
                    twist_cmd.linear.x = current_target_speed;
                    twist_cmd.angular.z = wz;
                }
                cmd.linear.y = 0.0; cmd.linear.z = 0.0;
                cmd.angular.x = 0.0; cmd.angular.y = 0.0;
                
                twist_cmd.linear.y = 0.0; twist_cmd.linear.z = 0.0;
                twist_cmd.angular.x = 0.0; twist_cmd.angular.y = 0.0;

                accel_pub->publish(cmd);
                cmd_vel_pub->publish(twist_cmd);
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