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
// zone_id -> 지난 tick의 imminent 차량 목록(거리순)
std::map<int, std::vector<int>> prev_imminent_vehicles;


// =========================
// 3개 Zone 정의 (Zone 4 제거)
// =========================
std::map<int, Pose> zones = {
  // origin
  // {1, {-2.333333333, 3.2}}, //
  // {2, {-2.333333333, 0.0}},
  // {3, {-2.333333333, -3.2}}
  // fix roi (0206 - main roi 변경으로 인해 zone 위치도 수정)
  {1, {-2.333333333, 3.5}}, // 3.55 -> 3.5
  {2, {-2.333333333, 0.0}},
  {3, {-2.333333333, -3.55}}
};

// =========================
// 4개 ROI 정의 (반지름 0.075m)
// =========================
std::map<int, Pose> rois = {
  {1, {1.43333333333, 1.2}},
  {2, {1.02329048013, 0.62933695117}},
  {3, {0.483, -0.23}},
  {4, {1.6, -1.0}}
};

const double ROI_RADIUS = 0.23;

// =========================
// CAV 색상 정의 (RGB)
// =========================
struct Color {
  double r, g, b, a;
};

std::map<int, Color> cav_colors = {
  {1, {0.0, 0.0, 1.0, 0.8}},  // 파란색
  {2, {0.0, 1.0, 0.0, 0.8}},  // 초록색
  {3, {0.5, 0.0, 0.5, 0.8}},  // 보라색
  {4, {1.0, 0.5, 0.0, 0.8}}   // 주황색
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

void cav_03_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    cav_poses[3] = {msg->pose.position.x, msg->pose.position.y};
}

void cav_04_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    cav_poses[4] = {msg->pose.position.x, msg->pose.position.y};
}
//hv추가
void hv_19_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  hv_poses[19] = {msg->pose.position.x, msg->pose.position.y};
}

void hv_20_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  hv_poses[20] = {msg->pose.position.x, msg->pose.position.y};
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
// Precollision Zone 체크 (반지름 0.7m)
// =========================
bool is_in_precollision_zone(Pose cav_pose, Pose zone_origin, double radius) {
    double distance = calculate_distance(cav_pose, zone_origin);
    return distance <= radius;
}

// =========================
// Imminent Collision Zone 체크 (반지름 0.5m)
// =========================
bool is_in_imminent_collision_zone(Pose cav_pose, Pose zone_origin, double radius) {
    double distance = calculate_distance(cav_pose, zone_origin);
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

  if (min_dist_found < overlap_threshold ) {
    // std::cout << "[PATH DEBUG] Min Distance: " << min_dist_found 
    //           << "m (Overlap: " << (is_overlapping ? "YES" : "NO") << ")" << std::endl;
  }

  return is_overlapping;
}

// =========================
// 차량이 Zone 중심에서 멀어지고 있는지 판단하는 함수
// =========================
bool is_moving_away(int cav_id, Pose zone_origin, const std::vector<Pose>& path) {
    if (path.empty()) return false;

    // 1. 현재 위치의 인덱스 찾기
    int current_idx = find_closest_waypoint_index(path, cav_poses[cav_id]);
    
    // 2. 미래 위치 가져오기
    int future_idx = std::min(current_idx + 30, static_cast<int>(path.size()) - 1);
    
    // 경로 끝에 도달했으면 멀어지는 것으로 간주
    if (current_idx >= static_cast<int>(path.size()) - 5) return true;

    Pose current_pose = path[current_idx];
    Pose future_pose = path[future_idx];

    // 3. 거리 비교
    double dist_current = calculate_distance(current_pose, zone_origin);
    double dist_future = calculate_distance(future_pose, zone_origin);

    // 미래의 거리가 현재 거리보다 크면 멀어지는 중이런식으로..
    return dist_future > dist_current;
}

// =========================
// Main CAV와 Precollision 후보군에서 Sub CAV 선택
// =========================
int select_best_sub_cav(int main_cav_id, const std::vector<int>& precollision_candidates, Pose zone_origin, double overlap_threshold, int lookahead_distance) {
  // 방어 로직: 빈 경로나 없는 CAV 체크
  if (cav_paths.find(main_cav_id) == cav_paths.end() || cav_paths[main_cav_id].empty()) {
    return -1;
  }
  if (cav_poses.find(main_cav_id) == cav_poses.end()) {
    return -1;
  }

  int main_idx = find_closest_waypoint_index(cav_paths[main_cav_id], cav_poses[main_cav_id]);
  std::vector<Pose> main_lookahead = get_lookahead_waypoints(cav_paths[main_cav_id], main_idx, lookahead_distance);

  std::vector<int> non_overlapping_candidates;
  for (int sub_id : precollision_candidates) {
    // 각 sub_id도 체크
    if (cav_paths.find(sub_id) == cav_paths.end() || cav_paths[sub_id].empty()) continue;
    if (cav_poses.find(sub_id) == cav_poses.end()) continue;
    
    int sub_idx = find_closest_waypoint_index(cav_paths[sub_id], cav_poses[sub_id]);
    std::vector<Pose> sub_lookahead = get_lookahead_waypoints(cav_paths[sub_id], sub_idx, lookahead_distance);

    double csv_distance = 0.0;
    if (!check_path_overlap(main_lookahead, sub_lookahead, overlap_threshold, csv_distance)) {
      non_overlapping_candidates.push_back(sub_id);
    }
  }

  // 경로 안 겹치는 후보군이 없을 때, 안전한 경로 겹침 체크
  if (non_overlapping_candidates.empty()) {
    // std::cout << "[SUB_CAV SELECT] No non-overlapping candidates. Checking safe overlap cases..." << std::endl;
    
    std::vector<int> safe_overlap_candidates;
    
    for (int sub_id : precollision_candidates) {
      int sub_idx = find_closest_waypoint_index(cav_paths[sub_id], cav_poses[sub_id]);
      std::vector<Pose> sub_lookahead = get_lookahead_waypoints(cav_paths[sub_id], sub_idx, lookahead_distance);
      
      // 겹침 인덱스 찾기
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
      
      // 겹침이 있으면 인덱스 차이 계산
      if (main_overlap_idx != -1 && sub_overlap_idx != -1) {
        int idx_diff = abs(main_overlap_idx - sub_overlap_idx);
        
        // std::cout << "[OVERLAP CHECK] CAV_" << main_cav_id << " vs CAV_" << sub_id 
        //           << " | Main_idx: " << main_overlap_idx 
        //           << ", Sub_idx: " << sub_overlap_idx 
        //           << ", Diff: " << idx_diff << std::endl;
        
        // Main의 앞쪽 점(큰 인덱스)과 Sub의 뒤쪽 점(작은 인덱스)이 만나면 안전
        // 또는 인덱스 차이가 충분히 크면 시간 여유가 있음
        if (idx_diff > 250 || (main_overlap_idx > 200 && sub_overlap_idx < 20)) {
          safe_overlap_candidates.push_back(sub_id);
          // std::cout << "[SAFE OVERLAP] CAV_" << sub_id << " added to safe candidates (idx_diff: " << idx_diff << ")" << std::endl;
        }
      }
    }
    
    // Safe overlap 후보군이 있으면 그 중에서 가장 가까운 차량 선택
    if (!safe_overlap_candidates.empty()) {
      int selected_sub = safe_overlap_candidates[0];
      double min_d = calculate_distance(cav_poses[selected_sub], zone_origin);
      for (int id : safe_overlap_candidates) {
        double d = calculate_distance(cav_poses[id], zone_origin);
        if (d < min_d) { min_d = d; selected_sub = id; }
      }
      // std::cout << "[SAFE OVERLAP] Selected CAV_" << selected_sub << " as Sub_CAV" << std::endl;
      return selected_sub;
    }
    
    // 안전한 경로 겹침도 없으면 -1 반환 (Main만 진행)
    // std::cout << "[SUB_CAV SELECT] No safe candidates found. Only Main_CAV will proceed." << std::endl;
    return -1;
  }

  int selected_sub = non_overlapping_candidates[0];
  double min_d = calculate_distance(cav_poses[selected_sub], zone_origin);
  for (int id : non_overlapping_candidates) {
    double d = calculate_distance(cav_poses[id], zone_origin);
    if (d < min_d) { min_d = d; selected_sub = id; }
  }
  return selected_sub;
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

  //(A) 매 프레임 RViz에 남아있는 이전 Marker를 확실히 정리
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = "map";
  clear.header.stamp = node->now();
  clear.ns = "clear_all";
  clear.id = 0;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear);

  // ========================================
  // 1. Zone 시각화 (Zone 1, 2, 3만)
  // ========================================
  for (const auto& [zone_id, zone_pos] : zones) {

    // Precollision Zone (노란색, 반투명)
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

    // Imminent Zone (빨간색, 반투명)
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
  // 2. ROI 시각화 (빨간색, 반투명, 작은 원)
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
    roi_marker.color.b = 0.0;
    roi_marker.color.a = 0.6;  // 반투명

    marker_array.markers.push_back(roi_marker);

    // ROI 텍스트 라벨
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
    roi_text.color.b = 0.0;
    roi_text.color.a = 1.0;

    marker_array.markers.push_back(roi_text);
  }

  // ========================================
  // 3. 각 CAV의 Lookahead 경로 시각화
  // ========================================
  for (int cav_id = 1; cav_id <= 4; cav_id++) {
    if (cav_poses.find(cav_id) == cav_poses.end()) continue;
    if (cav_paths[cav_id].empty()) continue;

    int current_idx = find_closest_waypoint_index(cav_paths[cav_id], cav_poses[cav_id]);
    std::vector<Pose> lookahead = get_lookahead_waypoints(cav_paths[cav_id], current_idx, visualization_lookahead);
    if (lookahead.empty()) continue;

    // LINE_STRIP 마커 생성
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

    // CAV 위치에 텍스트 표시 (CAV ID)
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
// Zone 모니터링 및 RED/GREEN_FLAG 제어 함수
// =========================
void monitor_zone(
    int zone_id,
    std::shared_ptr<rclcpp::Node> node,
    std::map<int, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr>& red_flag_pubs,
    std::map<int, std::vector<int>>& prev_red_flag_vehicles,
    double precollision_radius,
    double imminent_collision_radius,
    double overlap_threshold,
    int lookahead_distance,
    const std::map<int, int>& cav_index_map_actual_to_idx,
    const std::map<int, int>& cav_index_map_idx_to_actual) {
  
  Pose zone_origin = zones[zone_id];
  std::vector<int> precollision_vehicles;
  
  std::vector<std::pair<double, int>> imminent_with_dist; // first는 거리, second는 차 id
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
 //지금 이 Zone에 들어온 Imminent 차량 목록이, 지난번에 비해 바뀌었을 때만 로그 찍는 부분 //리스트의 순서나 구성이 바뀌면 로그출력
std::sort(imminent_with_dist.begin(), imminent_with_dist.end()); //정렬

std::vector<int> imminent_vehicles;
for (const auto& p : imminent_with_dist) {
  imminent_vehicles.push_back(p.second);
}

// 이전 imminent랑 비교해야 함
if (imminent_vehicles != prev_imminent_vehicles[zone_id]) { //prev_imminent_vehicles가 map<int, vector<int>> key: zone_id , value: 그 zone의 “이전 imminent 리스트”
  std::string imminent_ids;
  for (int id : imminent_vehicles) imminent_ids += std::to_string(id) + " ";

  if (!imminent_vehicles.empty()) {
    // RCLCPP_WARN(node->get_logger(),
    //             "[ZONE_%d IMMINENT] CAVs (Closest First): %s",
    //             zone_id, imminent_ids.c_str());
  } else {
    // RCLCPP_INFO(node->get_logger(),
    //             "[ZONE_%d IMMINENT] EMPTY",
    //             zone_id);
  }

  prev_imminent_vehicles[zone_id] = imminent_vehicles;
}

//
  std::vector<int> current_red_flag_vehicles; //이번 실행에서 멈춰야하는 차량  id 목록

  if (imminent_vehicles.empty()) {
    // RCLCPP_INFO(node->get_logger(), "[ZONE_%d] Imminent EMPTY", zone_id);
  } else {
    main_cav_id = imminent_vehicles[0]; //아까 imminent_vehicles 거리순으로 정렬해놓음 그래서 젤 가까운게 0번 인덱스
    
    // 방어 로직: main_cav_id의 경로가 존재하는지 확인
    if (cav_paths.find(main_cav_id) != cav_paths.end() && !cav_paths[main_cav_id].empty()) {
      // RCLCPP_INFO(node->get_logger(), "[ZONE_%d MAIN_CAV] Selected Closest CAV_%d (Dist: %.2fm)", 
      //             zone_id, main_cav_id, imminent_with_dist[0].first);

      if (!precollision_vehicles.empty()) {
        std::vector<int> sub_candidates;
        for (int cav_id : precollision_vehicles) {
          if (cav_id == main_cav_id) continue;
          if (cav_paths.find(cav_id) == cav_paths.end() || cav_paths[cav_id].empty()) continue;  // 경로 체크
          if (is_moving_away(cav_id, zone_origin, cav_paths[cav_id])) continue;
          
          sub_candidates.push_back(cav_id);
        }

        if (!sub_candidates.empty()) {
          cav_sub = select_best_sub_cav(main_cav_id, sub_candidates, zone_origin, overlap_threshold, lookahead_distance);

          if (cav_sub != -1) {
            // RCLCPP_INFO(node->get_logger(), "[ZONE_%d SUB_CAV] Selected Safe CAV_%d", zone_id, cav_sub);
          } else {
            // RCLCPP_ERROR(node->get_logger(), "[ZONE_%d] All candidates overlap! No Sub CAV selected.", zone_id);
          }

          for (int cav_id : sub_candidates) {
          if (cav_id == cav_sub) {
            continue; 
          } else {
            current_red_flag_vehicles.push_back(cav_id);
          }
        }
      }
    }
    } // main_cav_id 경로 체크 끝
  }

  if (current_red_flag_vehicles != prev_red_flag_vehicles[zone_id]) {
    RCLCPP_WARN(node->get_logger(), "[ZONE_%d RED_FLAG STATE CHANGED]", zone_id);

    for (int prev_cav_index : prev_red_flag_vehicles[zone_id]) {
      if (std::find(current_red_flag_vehicles.begin(), current_red_flag_vehicles.end(), prev_cav_index) == current_red_flag_vehicles.end()) {
        // Convert cav_index to actual_cav_id using reverse map
        if (cav_index_map_idx_to_actual.count(prev_cav_index)) {
          int prev_actual_cav_id = cav_index_map_idx_to_actual.at(prev_cav_index);
          auto msg = std_msgs::msg::Int32();
          msg.data = 0;  //0이 그린플래그
          if(red_flag_pubs.count(prev_actual_cav_id)) {
            red_flag_pubs[prev_actual_cav_id]->publish(msg);
            // RCLCPP_INFO(node->get_logger(), "[ZONE_%d GREEN_FLAG] CAV_%d -> RESUME", zone_id, prev_actual_cav_id);
          }
        }
      }
    }

    prev_red_flag_vehicles[zone_id] = current_red_flag_vehicles;
  }

  // 매 루프마다 현재 상태의 RED_FLAG 지속적으로 전송 (상태 유지)
  for (int cav_index : current_red_flag_vehicles) {
    // Convert cav_index to actual_cav_id using reverse map
    if (cav_index_map_idx_to_actual.count(cav_index)) {
      int actual_cav_id = cav_index_map_idx_to_actual.at(cav_index);
      auto msg = std_msgs::msg::Int32();
      msg.data = 1;  
      if(red_flag_pubs.count(actual_cav_id)) {
        red_flag_pubs[actual_cav_id]->publish(msg);
      }
    }
  }
  
  // 상태 변화 시에만 로그
  if (current_red_flag_vehicles != prev_red_flag_vehicles[zone_id]) {
    for (int cav_index : current_red_flag_vehicles) {
      if (cav_index_map_idx_to_actual.count(cav_index)) {
        int actual_cav_id = cav_index_map_idx_to_actual.at(cav_index);
        RCLCPP_WARN(node->get_logger(), "[ZONE_%d RED_FLAG] CAV_%d (Index=%d) -> STOP (Main=Index_%d, Sub=Index_%d)", 
                    zone_id, actual_cav_id, cav_index, main_cav_id, cav_sub);
      }
    }
  }
}

// =========================
// Main
// =========================
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("control_tower");

  double precollision_radius = 2.3;  // ** 2.3 -> 2.5
  double imminent_collision_radius = 1.8; // ** 1.8 -> 2.0
  double overlap_threshold = 0.2; // ** 0.2 -> 0.5 -> 0.7
  int lookahead_distance = 90;  // 경로 겹침 체크용 lookahead 거리
  int visualization_lookahead = 90;  // 시각화용 lookahead 거리
  
  // Get TEAM_AIM_HOME environment variable (default: /root/TEAM_AIM)
  const char* team_aim_home = std::getenv("TEAM_AIM_HOME");
  std::string base_path = (team_aim_home != nullptr && strlen(team_aim_home) > 0) 
                          ? std::string(team_aim_home) 
                          : std::string("/root/TEAM_AIM");
  std::string path_dir = base_path + "/src/global_path/";

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

  RCLCPP_INFO(node->get_logger(), "Loading CSV path files (from CAV_IDS mapping)...");
  for (size_t i = 0; i < active_cav_ids.size() && i < 4; i++) {
      int actual_cav_id = active_cav_ids[i];
      int cav_index = (int)i + 1;  // 1-indexed
      std::string cav_index_str = std::string(2 - std::to_string(cav_index).length(), '0') + std::to_string(cav_index);
      std::string csv_path = path_dir + "path_mission3_" + cav_index_str + ".csv";
      cav_paths[cav_index] = load_csv_file(csv_path);
      RCLCPP_INFO(node->get_logger(), "CAV_Index_%d (Actual_ID=%d): %zu waypoints loaded", cav_index, actual_cav_id, cav_paths[cav_index].size());
  }

  // RED_FLAG Publisher (based on active CAV count)
  std::map<int, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> red_flag_pubs;
  std::map<int, int> cav_index_map_actual_to_idx;  // Map actual_cav_id to cav_index
  std::map<int, int> cav_index_map_idx_to_actual;  // Map cav_index to actual_cav_id (reverse)
  for (size_t i = 0; i < active_cav_ids.size() && i < 4; i++) {
      int cav_index = (int)i + 1;  // 1-indexed
      int actual_cav_id = active_cav_ids[i];  // actual CAV ID from CAV_IDS
      cav_index_map_actual_to_idx[actual_cav_id] = cav_index;  // Store mapping for CSV lookup
      cav_index_map_idx_to_actual[cav_index] = actual_cav_id;  // Store reverse mapping for RED_FLAG publishing
      const std::string cav_id_str = std::string(2 - std::to_string(actual_cav_id).length(), '0') + std::to_string(actual_cav_id);
      const std::string flag_topic = "/CAV_" + cav_id_str + "_RED_FLAG";
      red_flag_pubs[actual_cav_id] = node->create_publisher<std_msgs::msg::Int32>(flag_topic, 1);
      RCLCPP_INFO(node->get_logger(), "Created RED_FLAG publisher for CAV_Index_%d (Actual_ID=%d): %s", cav_index, actual_cav_id, flag_topic.c_str());
  }

  //  Visualization Publisher 추가 
  auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/control_tower/visualization", 10);

  // Subscribers for CAV indices (1-4)
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

  rclcpp::Rate r(50);
  std::map<int, std::vector<int>> prev_red_flag_vehicles;

  RCLCPP_INFO(node->get_logger(), "Control Tower Node started! (Zone 1,2,3 + ROI 1,2,3,4)");

  while (rclcpp::ok()) {
    //  Zone 1, 2, 3만 모니터링 (Zone 4 제거) 
    for (int zone_id = 1; zone_id <= 3; zone_id++) {
      monitor_zone(zone_id, node, red_flag_pubs, prev_red_flag_vehicles,
                   precollision_radius, imminent_collision_radius, overlap_threshold, lookahead_distance, cav_index_map_actual_to_idx, cav_index_map_idx_to_actual);
    }
    // 시각화 발행 (10Hz) 
    publish_visualization(node, marker_pub, precollision_radius, imminent_collision_radius, visualization_lookahead);

    rclcpp::spin_some(node);
    r.sleep();
  }

  rclcpp::shutdown();
  return 0;
}