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
  double max_yaw_rate{5.5};

  double prev_x{0.0};
  double prev_y{0.0};
  double prev_yaw{0.0};
  bool has_prev{false};

  int red_flag{0};

  // 타워 모드 변수
  bool tower_mode{false};
  double target_linear_x{0.0};
};

// =========================
// CAV Structures (lap/finish)
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

  // 첫 줄도 데이터일 수 있어서 동일 포맷이면 읽음
  if (std::getline(file, line)) {
    std::stringstream ss(line);
    double x, y; char comma;
    if (ss >> x >> comma >> y) out.push_back({x, y});
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
  double gain_ld = 0.4;
  double max_ld  = 0.355;
  double min_ld  = 0.15;

  double velocity = st.speed_mps;
  double ld = gain_ld * velocity;
  st.lookahead_m = std::max(min_ld, std::min(max_ld, ld));
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

int findWaypoint(const vector<integrate_path_struct>& path, double x_m, double y_m, const double L_d) {
  if (path.empty()) { std::cout << "Path Empty" << std::endl; return -1; }
  int closest_idx = findClosestPoint(path, x_m, y_m);
  for (int i = closest_idx; i < (int)path.size(); ++i) {
    double d = hypot(path[i].x - x_m, path[i].y - y_m);
    if (d > L_d) return i;
  }
  return (int)path.size() - 1;
}

double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

bool isCorner(const vector<integrate_path_struct>& path, double /*L_d*/, int closest_idx) {
  int future_offset = 20;
  int path_size = (int)path.size();
  if (closest_idx < 0) return false;
  if (closest_idx + future_offset + 1 >= path_size) return false;

  double cur_ang = atan2(path[closest_idx + 1].y - path[closest_idx].y,
                         path[closest_idx + 1].x - path[closest_idx].x);

  int idx_future_start = closest_idx + future_offset;
  double fut_ang = atan2(path[idx_future_start + 1].y - path[idx_future_start].y,
                         path[idx_future_start + 1].x - path[idx_future_start + 1].x); // <-- (오타 방지용 자리)
  // 위 줄은 잘못된 예시가 될 수 있어, 아래 올바른 줄로 다시 계산
  fut_ang = atan2(path[idx_future_start + 1].y - path[idx_future_start].y,
                  path[idx_future_start + 1].x - path[idx_future_start].x);

  double diff = fabs(normalizeAngle(fut_ang - cur_ang));
  if (diff > M_PI / 2) diff = M_PI - diff;

  double threshold_deg = 10.0;
  double threshold_rad = threshold_deg * (M_PI / 180.0);
  return (diff > threshold_rad);
}

void planVelocity(ControllerState& st, bool corner) {
  if (!corner) st.speed_mps = 2.0;
  else         st.speed_mps = 1.7;
}

// =========================
// Lap/Finish logic (내 차량만)
// =========================
void PoseCallbackForLap(const geometry_msgs::msg::PoseStamped::SharedPtr msg, CavState& current_cav) {
  double current_x = msg->pose.position.x;
  double current_y = msg->pose.position.y;

  if (!current_cav.is_initialized) {
    current_cav.start_x = current_x;
    current_cav.start_y = current_y;
    current_cav.is_initialized = true;
    current_cav.is_in_zone = true;
    std::cout << "[CAV " << current_cav.id << "] Start Point Set: ("
              << current_x << ", " << current_y << ")" << std::endl << std::endl;
    return;
  }

  if (current_cav.finished) return;

  double dist_to_start = std::hypot(current_x - current_cav.start_x, current_y - current_cav.start_y);

  if (dist_to_start < 0.1) {
    if (!current_cav.is_in_zone) {
      current_cav.current_lap += 1;
      current_cav.is_in_zone = true;
      std::cout << "[CAV " << current_cav.id << "] Lap Increased! Current Lap: "
                << current_cav.current_lap << std::endl << std::endl;

      if (current_cav.current_lap >= 5) {
        current_cav.finished = true;
        std::cout << "[CAV " << current_cav.id << "] FINISHED 5 LAPS!" << std::endl << std::endl;
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

  // ---- CAV ID
  int cav_id_default = readCavIdFromEnvOrDefault(1);
  node->declare_parameter<int>("cav_id", cav_id_default);
  const int cav_id = node->get_parameter("cav_id").as_int();
  const std::string my_id_str = twoDigitId(cav_id);

  // ---- Topics
  const std::string my_pose_topic    = "/CAV_" + my_id_str;               // 내 pose만 구독
  const std::string accel_topic      = "/CAV_" + my_id_str + "_accel";
  const std::string red_flag_topic   = "/CAV_" + my_id_str + "_RED_FLAG";
  const std::string target_vel_topic = "/CAV_" + my_id_str + "_target_vel";
  // const std::string cmd_vel_topic    = "/CAV_" + my_id_str + "/cmd_vel";
  const std::string cmd_vel_topic    = "/cmd_vel";

  RCLCPP_INFO(node->get_logger(), "My CAV_ID=%d", cav_id);

  // ---- Params
  node->declare_parameter<double>("speed_mps", 0.5);
  node->declare_parameter<double>("lookahead_m", 0.4);
  node->declare_parameter<double>("max_yaw_rate", 5.5);

  // [핵심] 경로를 mission1_01로 고정
  node->declare_parameter<std::string>(
      "path_csv",
      "/root/TEAM_AIM/src/global_path/path_mission1_01.csv"
  );

  st->speed_mps    = node->get_parameter("speed_mps").as_double();
  st->lookahead_m  = node->get_parameter("lookahead_m").as_double();
  st->max_yaw_rate = node->get_parameter("max_yaw_rate").as_double();

  const std::string path_csv = node->get_parameter("path_csv").as_string();
  if (!loadPathCsv(path_csv, integrate_path_vector)) {
    RCLCPP_FATAL(node->get_logger(), "Failed to load path csv: %s", path_csv.c_str());
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Loaded path: %s (%zu waypoints)", path_csv.c_str(), integrate_path_vector.size());

  // ---- Lap state (내 차량만)
  CavState my_cav {cav_id, 0.0, 0.0, 0, false, false, false};

  // ---- Pub/Sub
  auto accel_pub = node->create_publisher<geometry_msgs::msg::Accel>(accel_topic, rclcpp::SensorDataQoS());
  auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, rclcpp::QoS(10));

  auto flag_sub = node->create_subscription<std_msgs::msg::Int32>(
      red_flag_topic, 50,
      [st](const std_msgs::msg::Int32::SharedPtr msg) { st->red_flag = msg->data; }
  );

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

  // ---- 내 차량 pose만 구독
  auto pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      my_pose_topic, rclcpp::SensorDataQoS(),
      [st, accel_pub, cmd_vel_pub, &my_cav](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
      {
        // 1) Lap/Finish 업데이트 (내 차량만)
        PoseCallbackForLap(msg, my_cav);

        // 완주 후엔 계속 정지 명령 유지
        if (my_cav.finished) {
          geometry_msgs::msg::Accel stop_cmd;
          stop_cmd.linear.x = 0.0; stop_cmd.linear.y = 0.0; stop_cmd.linear.z = 0.0;
          stop_cmd.angular.x = 0.0; stop_cmd.angular.y = 0.0; stop_cmd.angular.z = 0.0;
          accel_pub->publish(stop_cmd);
          geometry_msgs::msg::Twist stop_twist;
          stop_twist.linear.x = 0.0; stop_twist.linear.y = 0.0; stop_twist.linear.z = 0.0;
          stop_twist.angular.x = 0.0; stop_twist.angular.y = 0.0; stop_twist.angular.z = 0.0;
          cmd_vel_pub->publish(stop_twist);
          return;
        }

        // 2) pose/yaw
        get_pose(msg, x_m, y_m, z_m, x_q, y_q, z_q, w_q);

        double yaw = yawFromQuat(msg->pose.orientation, st->prev_yaw);
        if (st->has_prev && std::hypot(x_m - st->prev_x, y_m - st->prev_y) > 1e-4) {
          yaw = std::atan2(y_m - st->prev_y, x_m - st->prev_x);
        }
        st->prev_x = x_m; st->prev_y = y_m; st->prev_yaw = yaw; st->has_prev = true;

        // 3) Pure Pursuit
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

        int target_path_idx = findWaypoint(integrate_path_vector, x_m, y_m, st->lookahead_m);
        if (target_path_idx < 0) return;

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

        // 4) Publish
        geometry_msgs::msg::Accel cmd;
        geometry_msgs::msg::Twist twist_cmd;
        if (st->red_flag == 1) {
          cmd.linear.x  = 0.0;
          cmd.angular.z = 0.0;
          twist_cmd.linear.x = 0.0;
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
  );

  RCLCPP_INFO(node->get_logger(), "Subscribed ONLY to %s", my_pose_topic.c_str());

  (void)pose_sub; (void)flag_sub; (void)vel_sub;

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
