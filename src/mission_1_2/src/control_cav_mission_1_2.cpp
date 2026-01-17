#include "mission1_2/control_tower.hpp"

double get_quaternion2euler(double x, double y, double z, double w) {
  double siny_cosp = 2 * (w * z + x * y);
  double cosy_sinp = 1 - 2 * (y * y + z * z);
  double yaw = std::atan2(siny_cosp, cosy_sinp);
  return yaw;
}

void get_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                double &x_m, double &y_m, double &z_m) {
    x_m = msg->pose.position.x;
    y_m = msg->pose.position.y;
    z_m = msg->pose.position.z;
}

bool is_inside_zone(double x, double y, const zone& z) {
    if (x >= z.min_x && x <= z.max_x && 
        y >= z.min_y && y <= z.max_y) {
        return true;
    } else {
        return false;
    }
}


// 여기서 내리는 제어 명령은 문제 1-1 에서의 제어 로직과 똑같은 제어가 들어가야 제어에 혼돈이 안생기지 않을까요
void send_command(vehicle_state* cav, double vel) {
  cav->target_vel = vel;

  geometry_msgs::msg::Accel cmd;
  cmd.linear.x = vel;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;
  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = 0.0;

  if(cav->id == 1 && cav1_accel_pub != nullptr) {
    cav1_accel_pub->publish(cmd); 
  }
  else if (cav->id == 2 && cav2_accel_pub != nullptr) {
    cav2_accel_pub->publish(cmd);
  }
}



// ==========================================
// 3. 핵심 로직 (다중 구역 처리)
// ==========================================

// vehicle_state* cav: vehicle_state의 위치만 받아오겠다. 그거는 cav로 명한다.
bool process_zone1(vehicle_state* cav) {
    zone& z = g_zones[0];
    int zone_num = 1;


    // 1. zone 안으로 차량이 들어왔는지 판단.
    if(is_inside_zone(cav->x_m, cav->y_m, z)) {
        // 2. 차량이 zone 안에 들어왔을 때
        // case1. 이미 다른 차가 점유 중인 경우
        if(z.is_occupied && z.occupied_id != cav->id) {
            send_command(cav, stop_speed);
        } 
        // case2. 아무도 없거나 내가 점유 중인 경우
        else {
            if(!z.is_occupied) {
                z.is_occupied = true;
                z.occupied_id = cav->id;
                std::cout << "\033[1;33m[Zone " << zone_num << "] CAV_" << cav->id 
                          << " 진입! (좌표: " << cav->x_m << ", " << cav->y_m << ")\033[0m" << std::endl;
            }
            send_command(cav, target_speed);
        }
        return true;
    }

    // 3. 점유 해제 로직. (존에서 나갈 때 green flag로 바꾸고 점유 인덱스 초기화)
    else {
        if (z.occupied_id == cav->id && !is_inside_zone(cav->x_m, cav->y_m, z)) {
            z.is_occupied = false;
            z.occupied_id = 0;
            std::cout << "\033[1;36m[Zone " << zone_num << "] CAV_" << cav->id 
                          << " 해제 완료! (좌표: " << cav->x_m << ", " << cav->y_m << ")\033[0m" << std::endl;
            send_command(cav, target_speed);
            return true;
        }
        return false;
    }
}

bool process_zone2(vehicle_state* cav) {
    zone& z = g_zones[1];
    int zone_num = 2;

    // 1. zone 안으로 차량이 들어왔는지 판단.
    if(is_inside_zone(cav->x_m, cav->y_m, z)) {
        // 2. 차량이 zone 안에 들어왔을 때
        // case1. 이미 다른 차가 점유 중인 경우
        if(z.is_occupied && z.occupied_id != cav->id) {
            send_command(cav, stop_speed);
        } 
        // case2. 아무도 없거나 내가 점유 중인 경우
        else {
            if(!z.is_occupied) {
                z.is_occupied = true;
                z.occupied_id = cav->id;
                std::cout << "\033[1;33m[Zone " << zone_num << "] CAV_" << cav->id 
                          << " 진입! (좌표: " << cav->x_m << ", " << cav->y_m << ")\033[0m" << std::endl;
            }
            send_command(cav, target_speed);
        }
        return true;
    }

    // 3. 점유 해제 로직. (존에서 나갈 때 green flag로 바꾸고 점유 인덱스 초기화)
    else {
        if (z.occupied_id == cav->id && !is_inside_zone(cav->x_m, cav->y_m, z)) {
            z.is_occupied = false;
            z.occupied_id = 0;
            std::cout << "\033[1;36m[Zone " << zone_num << "] CAV_" << cav->id 
                          << " 해제 완료! (좌표: " << cav->x_m << ", " << cav->y_m << ")\033[0m" << std::endl;
            send_command(cav, target_speed);
            return true;
        }
        return false;
    }
}

bool process_zone3(vehicle_state* cav) {
    zone& z = g_zones[2];
    int zone_num = 3;

    // 1. zone 안으로 차량이 들어왔는지 판단.
    if(is_inside_zone(cav->x_m, cav->y_m, z)) {
        // 2. 차량이 zone 안에 들어왔을 때
        // case1. 이미 다른 차가 점유 중인 경우
        if(z.is_occupied && z.occupied_id != cav->id) {
            send_command(cav, stop_speed);
        } 
        // case2. 아무도 없거나 내가 점유 중인 경우
        else {
            if(!z.is_occupied) {
                z.is_occupied = true;
                z.occupied_id = cav->id;
                std::cout << "\033[1;33m[Zone " << zone_num << "] CAV_" << cav->id 
                          << " 진입! (좌표: " << cav->x_m << ", " << cav->y_m << ")\033[0m" << std::endl;
            }
            send_command(cav, target_speed);
        }
        return true;
    }

    // 3. 점유 해제 로직. (존에서 나갈 때 green flag로 바꾸고 점유 인덱스 초기화)
    else {
        if (z.occupied_id == cav->id && !is_inside_zone(cav->x_m, cav->y_m, z)) {
            z.is_occupied = false;
            z.occupied_id = 0;
            std::cout << "\033[1;36m[Zone " << zone_num << "] CAV_" << cav->id 
                          << " 해제 완료! (좌표: " << cav->x_m << ", " << cav->y_m << ")\033[0m" << std::endl;
            send_command(cav, target_speed);
            return true;
        }
        return false;
    }
}

bool process_zone4(vehicle_state* cav) {
    zone& z = g_zones[3];
    int zone_num = 4;

    // 1. zone 안으로 차량이 들어왔는지 판단.
    if(is_inside_zone(cav->x_m, cav->y_m, z)) {
        // 2. 차량이 zone 안에 들어왔을 때
        // case1. 이미 다른 차가 점유 중인 경우
        if(z.is_occupied && z.occupied_id != cav->id) {
            send_command(cav, stop_speed);
        } 
        // case2. 아무도 없거나 내가 점유 중인 경우
        else {
            if(!z.is_occupied) {
                z.is_occupied = true;
                z.occupied_id = cav->id;
                std::cout << "\033[1;33m[Zone " << zone_num << "] CAV_" << cav->id 
                          << " 진입! (좌표: " << cav->x_m << ", " << cav->y_m << ")\033[0m" << std::endl;
            }
            send_command(cav, target_speed);
        }
        return true;
    }

    // 3. 점유 해제 로직. (존에서 나갈 때 green flag로 바꾸고 점유 인덱스 초기화)
    else {
        if (z.occupied_id == cav->id && !is_inside_zone(cav->x_m, cav->y_m, z)) {
            z.is_occupied = false;
            z.occupied_id = 0;
            std::cout << "\033[1;36m[Zone " << zone_num << "] CAV_" << cav->id 
                          << " 해제 완료! (좌표: " << cav->x_m << ", " << cav->y_m << ")\033[0m" << std::endl;
            send_command(cav, target_speed);
            return true;
        }
        return false;
    }
}

void on_vehicle_update(vehicle_state* cav) {
    if(process_zone1(cav)) {return;}

    if(process_zone2(cav)) {return;}

    if(process_zone3(cav)) {return;}

    if(process_zone4(cav)) {return;}

    send_command(cav, target_speed);
}

void cav1_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    double x_1, y_1, z_1;

    get_pose(msg, x_1, y_1, z_1);

    g_cav1.x_m = x_1;
    g_cav1.y_m = y_1;
    g_cav1.z_m = z_1;
    
    double x = msg->pose.orientation.x;
    double y = msg->pose.orientation.y;
    double z = msg->pose.orientation.z;
    double w = msg->pose.orientation.w;

    double yaw = get_quaternion2euler(x, y, z, w);
    g_cav1.yaw_rad = yaw;

    on_vehicle_update(&g_cav1);
}

void cav2_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    double x_2, y_2, z_2;

    get_pose(msg, x_2, y_2, z_2);

    g_cav2.x_m = x_2;
    g_cav2.y_m = y_2;
    g_cav2.z_m = z_2;
    
    double x = msg->pose.orientation.x;
    double y = msg->pose.orientation.y;
    double z = msg->pose.orientation.z;
    double w = msg->pose.orientation.w;

    double yaw = get_quaternion2euler(x, y, z, w);
    g_cav2.yaw_rad = yaw;

    on_vehicle_update(&g_cav2);
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("control_tower_node");

  cav1_accel_pub = node->create_publisher<geometry_msgs::msg::Accel>("/CAV_01_accel", 10);
  cav2_accel_pub = node->create_publisher<geometry_msgs::msg::Accel>("/CAV_02_accel", 10);

//   cav1_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/Car_pose", 10);
//   cav2_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>("/Car_pose", 10);
  auto sub_1 = node->create_subscription<geometry_msgs::msg::PoseStamped>("/CAV_01", rclcpp::SensorDataQoS(), cav1_callback);
  auto sub_2 = node->create_subscription<geometry_msgs::msg::PoseStamped>("/CAV_02", rclcpp::SensorDataQoS(), cav2_callback);
  std::cout << "Localization Node가 시작되었습니다." << std::endl;
  
  rclcpp::spin(node);
  return 0;
}