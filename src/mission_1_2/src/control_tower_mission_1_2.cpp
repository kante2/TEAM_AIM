#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cmath>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <limits>
#include <fstream>
#include <sstream>

using namespace std;

// =========================
// Pose Struct
// =========================
struct Pose {
  double x;
  double y;
};

// =========================
// Global Variables
// =========================
std::map<int, Pose> cav_poses;
std::map<int, std::vector<Pose>> cav_paths;
std::map<int, Pose> hv_poses;
std::map<int, std::vector<int>> prev_imminent_vehicles;

// =========================
// 2개 Zone 정의 (Zone 2, 4만)
// =========================
std::map<int, Pose> zones = {
  {2, {-2.333333333, 0.0}},
  {4, {2.001666784286499, 2.7}}
};

// =========================
// 4개 ROI 정의 (CAV1: ROI1,2 / CAV2: ROI3)
// =========================
std::map<int, Pose> rois = {
  {1, {1.1329693794250488, -0.5619539022445679}},  // CAV1이 먼저 지나는 ROI
  {2, {2.125271797180176, -0.6247305870056152}},   // CAV1이 두 번째로 지나는 ROI
  {3, {1.774999976158142, -1.8}},      // CAV2가 지나는 ROI
  {4, {5.05833333333333, 0.666666666666667}} // 합류 차선 roi
};

const double ROI_RADIUS = 0.23;       // ROI 1, 2, 3 (기존 크기)
const double ROI_MERGE_RADIUS = 0.6;  // ROI 4 (합류 구간용, 더 크게 설정)
// =========================
// CAV 색상 정의 (CAV 1,2만 사용)
// =========================
struct Color {
  double r, g, b, a;
};

std::map<int, Color> cav_colors = {
  {1, {0.0, 0.0, 1.0, 0.8}},  // 파란색
  {2, {0.0, 1.0, 0.0, 0.8}}   // 초록색
};

// =========================
// CAV Pose 콜백 함수들
// =========================
void cav_01_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    cav_poses[1] = {msg->pose.position.x, msg->pose.position.y};
}

void cav_02_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    cav_poses[2] = {msg->pose.position.x, msg->pose.position.y};
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
// Precollision Zone 체크
// =========================
bool is_in_precollision_zone(Pose cav_pose, Pose zone_origin, double radius) {
    double distance = calculate_distance(cav_pose, zone_origin);
    return distance <= radius;
}

// =========================
// Imminent Collision Zone 체크
// =========================
bool is_in_imminent_collision_zone(Pose cav_pose, Pose zone_origin, double radius) {
    double distance = calculate_distance(cav_pose, zone_origin);
    return distance <= radius;
}

// =========================
// ROI 체크 함수
// =========================
bool is_in_roi(Pose cav_pose, Pose roi_origin, double radius) {
    double distance = calculate_distance(cav_pose, roi_origin);
    return distance <= radius;
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

// =========================
// 현재 위치에서 가장 가까운 웨이포인트 인덱스 찾기
// =========================
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
// 현재 인덱스부터 lookahead_count만큼의 웨이포인트 추출
// =========================
std::vector<Pose> get_lookahead_waypoints(
    const std::vector<Pose>& path,
    int current_idx,
    int lookahead_count = 200) {

  std::vector<Pose> lookahead_points;

  if (path.empty() || current_idx < 0) {
    return lookahead_points;
  }

  int end_idx = std::min(current_idx + lookahead_count, static_cast<int>(path.size()));

  for (int i = current_idx; i < end_idx; i++) {
    lookahead_points.push_back(path[i]);
  }

  return lookahead_points;
}

// =========================
// 경로 겹침 판단 함수
// =========================
bool check_path_overlap(
    const std::vector<Pose>& main_lookahead,
    const std::vector<Pose>& sub_lookahead,
    double overlap_threshold,
    double& csv_distance) {

  if (main_lookahead.empty() || sub_lookahead.empty()) {
    return false;
  }

  double min_dist_found = std::numeric_limits<double>::max();
  bool is_overlapping = false;

  for (const auto& main_point : main_lookahead) {
    for (const auto& sub_point : sub_lookahead) {
      double dist = calculate_distance(main_point, sub_point);
      
      if (dist < min_dist_found) {
        min_dist_found = dist;
      }

      if (dist < overlap_threshold) {
        is_overlapping = true;
      }
    }
  }

  csv_distance = min_dist_found;

  if (min_dist_found < overlap_threshold) {
    std::cout << "[PATH DEBUG] Min Distance: " << min_dist_found 
              << "m (Overlap: " << (is_overlapping ? "YES" : "NO") << ")" << std::endl;
  }

  return is_overlapping;
}

// =========================
// 차량이 Zone 중심에서 멀어지고 있는지 판단하는 함수
// =========================
bool is_moving_away(int cav_id, Pose zone_origin, const std::vector<Pose>& path) {
    if (path.empty()) return false;

    int current_idx = find_closest_waypoint_index(path, cav_poses[cav_id]);
    int future_idx = std::min(current_idx + 30, static_cast<int>(path.size()) - 1);
    
    if (current_idx >= static_cast<int>(path.size()) - 5) return true;

    Pose current_pose = path[current_idx];
    Pose future_pose = path[future_idx];

    double dist_current = calculate_distance(current_pose, zone_origin);
    double dist_future = calculate_distance(future_pose, zone_origin);

    return dist_future > dist_current;
}

// =========================
// 2대 차량용 간소화된 Sub CAV 선택 함수
// =========================
int select_sub_cav_for_two_vehicles(
    int main_cav_id, 
    const std::vector<int>& precollision_candidates,
    Pose zone_origin,
    double overlap_threshold,
    int lookahead_distance) {
  
  if (precollision_candidates.empty()) {
    return -1;
  }
  
  if (precollision_candidates.size() == 1) {
    int sub_id = precollision_candidates[0];
    
    if (cav_paths[main_cav_id].empty() || cav_paths[sub_id].empty()) {
      return -1;
    }

    int main_idx = find_closest_waypoint_index(cav_paths[main_cav_id], cav_poses[main_cav_id]);
    int sub_idx = find_closest_waypoint_index(cav_paths[sub_id], cav_poses[sub_id]);
    
    std::vector<Pose> main_lookahead = get_lookahead_waypoints(cav_paths[main_cav_id], main_idx, lookahead_distance);
    std::vector<Pose> sub_lookahead = get_lookahead_waypoints(cav_paths[sub_id], sub_idx, lookahead_distance);

    double csv_distance = 0.0;
    bool overlaps = check_path_overlap(main_lookahead, sub_lookahead, overlap_threshold, csv_distance);
    
    if (!overlaps) {
      std::cout << "[SUB_CAV] CAV_" << sub_id << " selected (no overlap)" << std::endl;
      return sub_id;
    }
    
    // 경로 겹침이 있으면 안전 여부 체크
    int main_overlap_idx = -1;
    int sub_overlap_idx = -1;
    double min_overlap_dist = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < main_lookahead.size(); i++) {
      for (size_t j = 0; j < sub_lookahead.size(); j++) {
        double dist = calculate_distance(main_lookahead[i], sub_lookahead[j]);
        if (dist < overlap_threshold && dist < min_overlap_dist) {
          min_overlap_dist = dist;
          main_overlap_idx = static_cast<int>(i);
          sub_overlap_idx = static_cast<int>(j);
        }
      }
    }
    
    if (main_overlap_idx != -1 && sub_overlap_idx != -1) {
      int idx_diff = abs(main_overlap_idx - sub_overlap_idx);
      
      std::cout << "[OVERLAP CHECK] Main_idx: " << main_overlap_idx 
                << ", Sub_idx: " << sub_overlap_idx 
                << ", Diff: " << idx_diff << std::endl;
      
      if (idx_diff > 150 || (main_overlap_idx > 170 && sub_overlap_idx < 30)) {
        std::cout << "[SAFE OVERLAP] CAV_" << sub_id << " approved" << std::endl;
        return sub_id;
      }
    }
    
    std::cout << "[UNSAFE OVERLAP] CAV_" << sub_id << " rejected" << std::endl;
    return -1;
  }
  
  return -1;
}

// =========================
// ROI 기반 CAV2 제어 로직 (상태 유지 방식 적용)
// =========================
bool should_cav2_stop_by_roi(std::shared_ptr<rclcpp::Node> node) {
  // 차량 존재 확인
  if (cav_poses.find(1) == cav_poses.end() || cav_poses.find(2) == cav_poses.end()) {
    return false;
  }

  Pose cav1_pose = cav_poses[1];
  Pose cav2_pose = cav_poses[2];

  // 현재 각 ROI까지의 거리 계산
  double dist_cav1_to_roi1 = calculate_distance(cav1_pose, rois[1]);
  double dist_cav1_to_roi2 = calculate_distance(cav1_pose, rois[2]);
  
  // CAV2가 ROI3(진입 대기 구역)에 있는지 확인
  bool cav2_in_roi3 = is_in_roi(cav2_pose, rois[3], ROI_RADIUS);

  // ==========================================
  // [수정된 핵심 로직] 상태 기억 (Latching)
  // ==========================================
  // static 변수를 사용하여 함수가 종료되어도 값을 기억하게 함
  // true: CAV1이 ROI 1을 지났고 아직 ROI 2를 안 지남 (교차로 점유 중)
  // false: 교차로 비어있음
  static bool cav1_is_crossing = false; 

  // 1. 진입 감지: CAV1이 ROI 1에 들어왔다면 -> 교차로 점유 시작 (Lock)
  if (dist_cav1_to_roi1 <= ROI_RADIUS) {
    if (!cav1_is_crossing) {
        RCLCPP_WARN(node->get_logger(), "[LATCH] CAV1 Entered ROI 1 -> CROSSING START");
        cav1_is_crossing = true;
    }
  }

  // 2. 탈출 감지: CAV1이 ROI 2에 도달했다면 -> 교차로 점유 해제 (Unlock)
  // ROI 2에 "도달"했을 때 해제하거나, 더 안전하게 하려면 "지나갔을 때" 해제해도 됩니다.
  // 여기서는 ROI 2 반경 안에 들어오면 안전하다고 판단하여 해제합니다.
  if (dist_cav1_to_roi2 <= ROI_RADIUS) {
    if (cav1_is_crossing) {
        RCLCPP_INFO(node->get_logger(), "[LATCH] CAV1 Reached ROI 2 -> CROSSING END");
        cav1_is_crossing = false;
    }
  }

  // 디버그 로그 (상태 확인용)
  static int debug_cnt = 0;
  if (debug_cnt++ % 25 == 0) {
    RCLCPP_INFO(node->get_logger(), 
                "[ROI STATE] C1_Crossing: %s | C2_in_ROI3: %s | D1: %.2f, D2: %.2f",
                cav1_is_crossing ? "YES (BUSY)" : "NO (FREE)",
                cav2_in_roi3 ? "YES" : "NO",
                dist_cav1_to_roi1, dist_cav1_to_roi2);
  }

  // 3. 최종 판단
  // CAV2가 대기 구역(ROI 3)에 있고 && CAV1이 교차로를 건너는 중이라면 -> 정지
  if (cav2_in_roi3 && cav1_is_crossing) {
    return true; // RED FLAG
  }

  return false; // GREEN FLAG
}

// =========================
// ROI 4 (합류 구간) 기반 CAV1 제어 로직 (반지름 확대 적용)
// =========================
bool should_cav1_stop_by_merge_roi(std::shared_ptr<rclcpp::Node> node) {
  if (cav_poses.find(1) == cav_poses.end() || cav_poses.find(2) == cav_poses.end()) {
    return false;
  }

  // ** 변경점: ROI 4 체크 시 ROI_MERGE_RADIUS 사용 **
  bool cav1_in_merge = is_in_roi(cav_poses[1], rois[4], ROI_MERGE_RADIUS); 
  bool cav2_in_merge = is_in_roi(cav_poses[2], rois[4], ROI_MERGE_RADIUS);

  static bool is_conflict_state = false; 

  // [LOCK] 두 차량이 큰 반경 안에 들어오면
  if (cav1_in_merge && cav2_in_merge) {
    if (!is_conflict_state) {
        RCLCPP_WARN(node->get_logger(), "[MERGE START] Both Vehicles in ROI 4 (Large Zone) -> LOCK");
        is_conflict_state = true;
    }
  }

  // [UNLOCK] CAV 2가 큰 반경을 완전히 벗어나면
  if (is_conflict_state && !cav2_in_merge) {
    RCLCPP_INFO(node->get_logger(), "[MERGE END] CAV 2 Left ROI 4 -> UNLOCK");
    is_conflict_state = false;
  }

  return is_conflict_state;
}

// =========================
// RViz 시각화 함수
// =========================
void publish_visualization(
    std::shared_ptr<rclcpp::Node> node,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub,
    double precollision_radius,
    double imminent_collision_radius,
    int visualization_lookahead) {

  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = "map";
  clear.header.stamp = node->now();
  clear.ns = "clear_all";
  clear.id = 0;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear);

  // ========================================
  // 1. Zone 시각화 (Zone 2, 4만)
  // ========================================
  for (const auto& [zone_id, zone_pos] : zones) {
    // Precollision Zone (노란색)
    visualization_msgs::msg::Marker precollision_marker;
    precollision_marker.header.frame_id = "map";
    precollision_marker.header.stamp = node->now();
    precollision_marker.ns = "precollision_zones";
    precollision_marker.id = 100 + zone_id;
    precollision_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    precollision_marker.action = visualization_msgs::msg::Marker::ADD;

    precollision_marker.pose.position.x = zone_pos.x;
    precollision_marker.pose.position.y = zone_pos.y;
    precollision_marker.pose.position.z = 0.0;
    precollision_marker.pose.orientation.w = 1.0;

    precollision_marker.scale.x = precollision_radius * 2.0;
    precollision_marker.scale.y = precollision_radius * 2.0;
    precollision_marker.scale.z = 0.01;

    precollision_marker.color.r = 1.0;
    precollision_marker.color.g = 1.0;
    precollision_marker.color.b = 0.0;
    precollision_marker.color.a = 0.3;

    marker_array.markers.push_back(precollision_marker);

    // Imminent Zone (빨간색)
    visualization_msgs::msg::Marker imminent_marker;
    imminent_marker.header.frame_id = "map";
    imminent_marker.header.stamp = node->now();
    imminent_marker.ns = "imminent_zones";
    imminent_marker.id = 200 + zone_id;
    imminent_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    imminent_marker.action = visualization_msgs::msg::Marker::ADD;

    imminent_marker.pose.position.x = zone_pos.x;
    imminent_marker.pose.position.y = zone_pos.y;
    imminent_marker.pose.position.z = 0.0;
    imminent_marker.pose.orientation.w = 1.0;

    imminent_marker.scale.x = imminent_collision_radius * 2.0;
    imminent_marker.scale.y = imminent_collision_radius * 2.0;
    imminent_marker.scale.z = 0.01;

    imminent_marker.color.r = 1.0;
    imminent_marker.color.g = 0.0;
    imminent_marker.color.b = 0.0;
    imminent_marker.color.a = 0.5;

    marker_array.markers.push_back(imminent_marker);
  }

  // ========================================
  // 2. ROI 시각화 (3개)
  // ========================================
  for (const auto& [roi_id, roi_pos] : rois) {
    visualization_msgs::msg::Marker roi_marker;
    roi_marker.header.frame_id = "map";
    roi_marker.header.stamp = node->now();
    roi_marker.ns = "roi_zones";
    roi_marker.id = 300 + roi_id;
    roi_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    roi_marker.action = visualization_msgs::msg::Marker::ADD;

    roi_marker.pose.position.x = roi_pos.x;
    roi_marker.pose.position.y = roi_pos.y;
    roi_marker.pose.position.z = 0.0;
    roi_marker.pose.orientation.w = 1.0;

    roi_marker.scale.x = ROI_RADIUS * 2.0;
    roi_marker.scale.y = ROI_RADIUS * 2.0;
    roi_marker.scale.z = 0.01;

    roi_marker.color.r = 1.0;
    roi_marker.color.g = 0.0;
    roi_marker.color.b = 1.0;
    roi_marker.color.a = 0.6;

    marker_array.markers.push_back(roi_marker);

    visualization_msgs::msg::Marker roi_text;
    roi_text.header.frame_id = "map";
    roi_text.header.stamp = node->now();
    roi_text.ns = "roi_labels";
    roi_text.id = 400 + roi_id;
    roi_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    roi_text.action = visualization_msgs::msg::Marker::ADD;

    roi_text.pose.position.x = roi_pos.x;
    roi_text.pose.position.y = roi_pos.y;
    roi_text.pose.position.z = 0.3;
    roi_text.pose.orientation.w = 1.0;

    roi_text.text = "ROI_" + std::to_string(roi_id);
    roi_text.scale.z = 0.25;

    roi_text.color.r = 1.0;
    roi_text.color.g = 0.0;
    roi_text.color.b = 1.0;
    roi_text.color.a = 1.0;

    marker_array.markers.push_back(roi_text);
  }

  // ========================================
  // 3. CAV 경로 시각화 (CAV 1,2만)
  // ========================================
  for (int cav_id = 1; cav_id <= 2; cav_id++) {
    if (cav_poses.find(cav_id) == cav_poses.end()) continue;
    if (cav_paths[cav_id].empty()) continue;

    int current_idx = find_closest_waypoint_index(cav_paths[cav_id], cav_poses[cav_id]);
    std::vector<Pose> lookahead = get_lookahead_waypoints(cav_paths[cav_id], current_idx, visualization_lookahead);
    if (lookahead.empty()) continue;

    visualization_msgs::msg::Marker path_marker;
    path_marker.header.frame_id = "map";
    path_marker.header.stamp = node->now();
    path_marker.ns = "lookahead_paths";
    path_marker.id = 1000 + cav_id;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;

    path_marker.scale.x = 0.05;

    Color color = cav_colors[cav_id];
    path_marker.color.r = color.r;
    path_marker.color.g = color.g;
    path_marker.color.b = color.b;
    path_marker.color.a = color.a;

    path_marker.pose.orientation.w = 1.0;

    path_marker.points.clear();
    path_marker.points.reserve(lookahead.size());

    for (const auto& point : lookahead) {
      geometry_msgs::msg::Point p;
      p.x = point.x;
      p.y = point.y;
      p.z = 0.1;
      path_marker.points.push_back(p);
    }

    marker_array.markers.push_back(path_marker);

    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = node->now();
    text_marker.ns = "cav_labels";
    text_marker.id = 2000 + cav_id;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;

    text_marker.pose.position.x = cav_poses[cav_id].x;
    text_marker.pose.position.y = cav_poses[cav_id].y;
    text_marker.pose.position.z = 0.6;
    text_marker.pose.orientation.w = 1.0;

    text_marker.text = "CAV_" + std::to_string(cav_id);
    text_marker.scale.z = 0.35;

    text_marker.color.r = color.r;
    text_marker.color.g = color.g;
    text_marker.color.b = color.b;
    text_marker.color.a = 1.0;

    marker_array.markers.push_back(text_marker);
  }

  // ========================================
  // 4. HV 시각화
  // ========================================
  for (const auto& [hv_id, hv_pose] : hv_poses) {
    visualization_msgs::msg::Marker hv_marker;
    hv_marker.header.frame_id = "map";
    hv_marker.header.stamp = node->now();
    hv_marker.ns = "hv_positions";
    hv_marker.id = 3000 + hv_id;
    hv_marker.type = visualization_msgs::msg::Marker::SPHERE;
    hv_marker.action = visualization_msgs::msg::Marker::ADD;

    hv_marker.pose.position.x = hv_pose.x;
    hv_marker.pose.position.y = hv_pose.y;
    hv_marker.pose.position.z = 0.15;
    hv_marker.pose.orientation.w = 1.0;

    hv_marker.scale.x = 0.3;
    hv_marker.scale.y = 0.3;
    hv_marker.scale.z = 0.3;

    hv_marker.color.r = 1.0;
    hv_marker.color.g = 0.0;
    hv_marker.color.b = 0.0;
    hv_marker.color.a = 1.0;

    marker_array.markers.push_back(hv_marker);
  }

  marker_pub->publish(marker_array);
}

// =========================
// Zone 모니터링 및 제어 함수
// =========================
void monitor_zone(
    int zone_id,
    std::shared_ptr<rclcpp::Node> node,
    std::map<int, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr>& red_flag_pubs,
    std::map<int, std::vector<int>>& prev_red_flag_vehicles,
    double precollision_radius,
    double imminent_collision_radius,
    double overlap_threshold,
    int lookahead_distance) {
  
  Pose zone_origin = zones[zone_id];
  std::vector<int> precollision_vehicles;
  std::vector<std::pair<double, int>> imminent_with_dist;
  int cav_sub = -1;
  int main_cav_id = -1;

  for (const auto& [cav_id, pose] : cav_poses) {
    double dist = calculate_distance(pose, zone_origin);
    
    if (dist <= precollision_radius) {
      precollision_vehicles.push_back(cav_id);
    }
    if (dist <= imminent_collision_radius) {
      imminent_with_dist.push_back({dist, cav_id});
    }
  }

  std::sort(imminent_with_dist.begin(), imminent_with_dist.end());

  std::vector<int> imminent_vehicles;
  for (const auto& p : imminent_with_dist) {
    imminent_vehicles.push_back(p.second);
  }

  if (imminent_vehicles != prev_imminent_vehicles[zone_id]) {
    std::string imminent_ids;
    for (int id : imminent_vehicles) imminent_ids += std::to_string(id) + " ";

    if (!imminent_vehicles.empty()) {
      RCLCPP_WARN(node->get_logger(),
                  "[ZONE_%d IMMINENT] CAVs (Closest First): %s",
                  zone_id, imminent_ids.c_str());
    } else {
      RCLCPP_INFO(node->get_logger(),
                  "[ZONE_%d IMMINENT] EMPTY",
                  zone_id);
    }

    prev_imminent_vehicles[zone_id] = imminent_vehicles;
  }

  std::vector<int> current_red_flag_vehicles;

  if (imminent_vehicles.empty()) {
    // Imminent 차량 없음
  } else {
    main_cav_id = imminent_vehicles[0];
    RCLCPP_INFO(node->get_logger(), "[ZONE_%d MAIN_CAV] Selected Closest CAV_%d (Dist: %.2fm)", 
                zone_id, main_cav_id, imminent_with_dist[0].first);

    if (!precollision_vehicles.empty()) {
      std::vector<int> sub_candidates;
      for (int cav_id : precollision_vehicles) {
        if (cav_id == main_cav_id) continue;
        if (is_moving_away(cav_id, zone_origin, cav_paths[cav_id])) continue;
        sub_candidates.push_back(cav_id);
      }

      if (!sub_candidates.empty()) {
        cav_sub = select_sub_cav_for_two_vehicles(
            main_cav_id, sub_candidates, zone_origin, 
            overlap_threshold, lookahead_distance);

        if (cav_sub != -1) {
          RCLCPP_INFO(node->get_logger(), "[ZONE_%d SUB_CAV] Selected CAV_%d", zone_id, cav_sub);
        } else {
          RCLCPP_ERROR(node->get_logger(), "[ZONE_%d] Candidate overlaps! No Sub CAV selected.", zone_id);
        }

        for (int cav_id : sub_candidates) {
          if (cav_id != cav_sub) {
            current_red_flag_vehicles.push_back(cav_id);
          }
        }
      }
    }
  }

  if (current_red_flag_vehicles != prev_red_flag_vehicles[zone_id]) {
    RCLCPP_WARN(node->get_logger(), "[ZONE_%d RED_FLAG STATE CHANGED]", zone_id);

    for (int prev_id : prev_red_flag_vehicles[zone_id]) {
      if (std::find(current_red_flag_vehicles.begin(), current_red_flag_vehicles.end(), prev_id) 
          == current_red_flag_vehicles.end()) {
        auto msg = std_msgs::msg::Int32();
        msg.data = 0;
        red_flag_pubs[prev_id]->publish(msg);
        RCLCPP_INFO(node->get_logger(), "[ZONE_%d GREEN_FLAG] CAV_%d -> RESUME", zone_id, prev_id);
      }
    }

    for (int cav_id : current_red_flag_vehicles) {
      auto msg = std_msgs::msg::Int32();
      msg.data = 1;
      red_flag_pubs[cav_id]->publish(msg);
      RCLCPP_WARN(node->get_logger(), "[ZONE_%d RED_FLAG] CAV_%d -> STOP (Main=CAV_%d, Sub=CAV_%d)", 
                  zone_id, cav_id, main_cav_id, cav_sub);
    }

    prev_red_flag_vehicles[zone_id] = current_red_flag_vehicles;
  }
}

// =========================
// Main
// =========================
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("control_tower");

  double precollision_radius = 2.3;
  double imminent_collision_radius = 1.6;
  double overlap_threshold = 0.2;
  int lookahead_distance = 100;
  int visualization_lookahead = 20;
  std::string path_dir = "/root/TEAM_AIM/src/global_path/";

  RCLCPP_INFO(node->get_logger(), "Loading CSV path files...");
  for (int i = 1; i <= 2; i++) {
    std::string csv_path = path_dir + "path_mission1_0" + std::to_string(i) + ".csv";
    cav_paths[i] = load_csv_file(csv_path);
    RCLCPP_INFO(node->get_logger(), "CAV_%d: %zu waypoints loaded", i, cav_paths[i].size());
  }

  std::map<int, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> red_flag_pubs;
  for (int i = 1; i <= 2; i++) {
    const std::string flag_topic = "/CAV_0" + std::to_string(i) + "_RED_FLAG";
    red_flag_pubs[i] = node->create_publisher<std_msgs::msg::Int32>(flag_topic, 50);
  }

  auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/control_tower/visualization", 10);

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
 

  rclcpp::Rate r(50);
  std::map<int, std::vector<int>> prev_red_flag_vehicles;

  RCLCPP_INFO(node->get_logger(), "Control Tower Node started! (Zone 2,4 + ROI 1,2,3)");

  while (rclcpp::ok()) {
    // ROI 기반 CAV2 제어 (실시간 위치 기반)
    bool cav2_should_stop = should_cav2_stop_by_roi(node);
    
    static bool prev_cav2_roi_stop = false;
    if (cav2_should_stop != prev_cav2_roi_stop) {
      auto msg = std_msgs::msg::Int32();
      msg.data = cav2_should_stop ? 1 : 0;
      red_flag_pubs[2]->publish(msg);
      
      if (cav2_should_stop) {
        RCLCPP_ERROR(node->get_logger(), "[ROI CONTROL] CAV_2 RED_FLAG by ROI logic");
      } else {
        RCLCPP_INFO(node->get_logger(), "[ROI CONTROL] CAV_2 GREEN_FLAG by ROI logic");
      }
      
      prev_cav2_roi_stop = cav2_should_stop;
    }

    // ---------------------------------------------------------
    // ROI 4 기반 CAV1 제어 (합류 구간)
    // ---------------------------------------------------------
    bool cav1_merge_stop = should_cav1_stop_by_merge_roi(node);
    static bool prev_cav1_merge_stop = false;

    // 상태가 변했을 때만 Publish (토픽 부하 방지)
    if (cav1_merge_stop != prev_cav1_merge_stop) {
        auto msg = std_msgs::msg::Int32();
        msg.data = cav1_merge_stop ? 1 : 0;
        red_flag_pubs[1]->publish(msg); // CAV1에게 정지 신호 보냄

        if (cav1_merge_stop) {
            RCLCPP_ERROR(node->get_logger(), "[MERGE CONTROL] CAV_1 RED_FLAG! (Conflict at ROI 4)");
        } else {
            RCLCPP_INFO(node->get_logger(), "[MERGE CONTROL] CAV_1 GREEN_FLAG (ROI 4 Clear)");
        }

        prev_cav1_merge_stop = cav1_merge_stop;
    }
    
    // Zone 2, 4만 모니터링
    monitor_zone(2, node, red_flag_pubs, prev_red_flag_vehicles,
                 precollision_radius, imminent_collision_radius, 
                 overlap_threshold, lookahead_distance);
    monitor_zone(4, node, red_flag_pubs, prev_red_flag_vehicles,
                 precollision_radius, imminent_collision_radius, 
                 overlap_threshold, lookahead_distance);
    
    publish_visualization(node, marker_pub, precollision_radius, 
                         imminent_collision_radius, visualization_lookahead);

    rclcpp::spin_some(node);
    r.sleep();
  }

  rclcpp::shutdown();
  return 0;
}