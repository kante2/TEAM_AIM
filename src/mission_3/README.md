# Mission 3: 다중 차량 협력 주행 및 원형 교차로 통과 (Multi-Vehicle Cooperative Navigation Through Rotary)

## 📋 개요

**Mission 3**은 최대 4대의 자동주행 차량(CAV 1~4)과 2대의 수동조종 차량(HV 19, 20)이 **원형 교차로(Roundabout)**를 안전하게 통과하는 복잡한 협력 주행 미션입니다. 

이 미션은 다음을 포함합니다:
- **3개의 제어 노드**로 구성된 분산 제어 시스템
- **다중 ROI 및 충돌 방지 영역** 관리
- **노란색 신호(Yellow Flag)** 기반 속도 감속
- **호 길이(Arc Length) 계산**을 통한 원형 교차로 분석
- **HV 속도 측정** 및 협력 제어

---

## 🎯 미션 목표

- **안전한 원형 교차로 통과**: 4대의 CAV가 교차로를 통과
- **HV와의 협력**: 2대의 HV와 적절한 거리 유지
- **충돌 회피**: 우선순위 기반의 ROI 진입 제어
- **속도 조정**: Yellow Flag를 통한 감속 구간 관리
- **완주 감지**: 모든 차량의 주행 완료 인식

---

## 🏗️ 주요 구조

### 📁 파일 구성
```
mission_3/
├── CMakeLists.txt           # CMake 빌드 설정
├── package.xml              # ROS2 패키지 메타데이터
├── launch/                  # ROS2 실행 설정
└── src/
    ├── control_cav_mission_3.cpp      # CAV 제어 노드
    ├── control_tower_mission_3.cpp    # 타워(제어 센터) 노드
    └── control_rotary_mission_3.cpp   # 원형 교차로 전용 제어 노드
```

### 📦 ROS2 노드

#### 1. `control_cav_mission_3` 노드
각 자동주행 차량의 경로 추적, 속도 제어, Yellow Flag 대응을 담당합니다.

#### 2. `control_tower_mission_3` 노드
모든 차량의 위치를 모니터링하고, 3개의 교차로 Zone에서 충돌 회피 신호를 발행합니다.

#### 3. `control_rotary_mission_3` 노드
HV(Human-driven Vehicle)의 속도를 측정하고, 원형 교차로 내 ROI 진입 허가를 결정합니다.

---

## 🔧 핵심 기능

### 1️⃣ **경로 구조 및 3개 Zone 정의**

```cpp
// 3개의 교차로 Zone (Zone 4는 제거됨)
std::map<int, Pose> zones = {
  {1, {-2.333333333, 3.2}},    // 상단 교차로
  {2, {-2.333333333, 0.0}},    // 중앙 교차로
  {3, {-2.333333333, -3.2}}    // 하단 교차로
};

// 4개의 ROI (원형 교차로 내부)
std::map<int, Pose> rois = {
  {1, {1.43333333333, 1.2}},       // CAV 1, 2 진입 ROI
  {2, {1.02329048013, 0.62933695117}},  // 교차로 내부
  {3, {0.483, -0.23}},              // CAV 3, 4 통과 ROI
  {4, {1.6, -1.0}}                  // 교차로 출구
};

const double ROI_RADIUS = 0.23;  // ROI 반지름
```

### 2️⃣ **Pure Pursuit 경로 추적 (최적화 버전)**

**파라미터 (Mission 1-2 대비 개선):**
- **`speed_mps`**: 기본 속도 (0.5 m/s)
- **`lookahead_m`**: 선행거리 (0.3 m)
- **`max_yaw_rate`**: 최대 회전 속도 (2.0 rad/s) ← **더 보수적**

**동적 선행거리 계산:**
```cpp
void GetLd(ControllerState& st) {
  double gain_ld = 0.6;      // 0.4 → 0.6 (더 민감하게)
  double max_ld  = 0.355;    // 변화 없음
  double min_ld  = 0.1;      // 0.15 → 0.1 (더 작게)
  double velocity = st.speed_mps;
  double ld = gain_ld * velocity;
  st.lookahead_m = max(min_ld, std::min(max_ld, ld));
}
```

### 3️⃣ **Yellow Flag 기반 속도 제어**

기존의 Red Flag(정지)에 더해, **Yellow Flag(감속)**이 추가됩니다:

```cpp
struct ControllerState {
  // ...
  int red_flag{0};        // Red Flag: 1 = 정지, 0 = 진행
  int yellow_flag{0};     // Yellow Flag: 1 = 감속, 0 = 정상
};

// Yellow ROI 정의 (감속 구간)
std::map<int, Pose> yellow_cav_rois = {
  {1, {1.1945931911468506, 2.152200222015381}},
  {2, {-0.4866666793823242, -0.10833333432674408}}
};
```

**제어 로직:**
- **Red Flag = 1**: 즉시 정지 (선속도 = 0)
- **Yellow Flag = 1**: 속도 50% 감소
- **Red Flag = 0 & Yellow Flag = 0**: 정상 속도

### 4️⃣ **호 길이(Arc Length) 계산**

원형 교차로 내에서 차량 간 거리를 호 길이로 계산합니다:

```cpp
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
```

이를 통해 원형 교차로 내 차량들 간의 **원 호 상의 거리**를 계산합니다.

### 5️⃣ **HV(수동 차량) 속도 측정**

HV의 위치 변화를 통해 실시간 속도를 계산합니다:

```cpp
struct HVState {
    int id;
    bool is_first_msg = true;
    double prev_x = 0.0, prev_y = 0.0;
    rclcpp::Time last_time;
    std::deque<double> vel_buffer;  // 최근 10개 속도의 이동평균
    const size_t buffer_size = 10;
};

// 속도 계산
double velocity = distance / time_delta;
// 노이즈 제거를 위해 이동평균 사용
vel_buffer.push_back(velocity);
if (vel_buffer.size() > buffer_size) vel_buffer.pop_front();
double measured_hv_vel = std::accumulate(vel_buffer.begin(), vel_buffer.end(), 0.0) 
                         / vel_buffer.size();
```

### 6️⃣ **충돌 방지 로직 (3-Zone 관리)**

타워 노드에서 3개의 교차로 각각에 대해:

```cpp
// Zone별 충돌 판단
bool is_in_precollision_zone(Pose cav_pose, Pose zone_origin, double radius=0.7m)
bool is_in_imminent_collision_zone(Pose cav_pose, Pose zone_origin, double radius=0.5m)

// 타워 발행:
/CAV_01_RED_FLAG, /CAV_02_RED_FLAG, /CAV_03_RED_FLAG, /CAV_04_RED_FLAG
/CAV_01_YELLOW_FLAG, ... (선택사항)
```

---

## 📡 ROS2 토픽

### CAV 노드 (구독)
| 토픽명 | 메시지 타입 | 설명 |
|--------|-----------|------|
| `/CAV_01` ~ `/CAV_04` | `geometry_msgs/msg/PoseStamped` | 각 CAV의 위치/자세 |
| `/CAV_01_RED_FLAG` ~ `/CAV_04_RED_FLAG` | `std_msgs/msg/Int32` | 타워의 정지 신호 |
| `/CAV_01_YELLOW_FLAG` ~ `/CAV_04_YELLOW_FLAG` | `std_msgs/msg/Int32` | 타워의 감속 신호 |
| `/CAV_01_target_vel` ~ `/CAV_04_target_vel` | `std_msgs/msg/Float64` | 타워의 목표 속도 |

### CAV 노드 (발행)
| 토픽명 | 메시지 타입 | 설명 |
|--------|-----------|------|
| `/CAV_01_accel` ~ `/CAV_04_accel` | `geometry_msgs/msg/Accel` | 계산된 명령 |

### HV 노드 (구독)
| 토픽명 | 메시지 타입 | 설명 |
|--------|-----------|------|
| `/HV_19`, `/HV_20` | `geometry_msgs/msg/PoseStamped` | HV 위치/자세 |

### 타워 노드 (발행)
| 토픽명 | 메시지 타입 | 설명 |
|--------|-----------|------|
| `/visualization_marker_array` | `visualization_msgs/msg/MarkerArray` | RViz 시각화 |

---

## 🚀 파라미터

### CAV 노드
```bash
ros2 run mission_3 control_cav_mission_3 \
  --ros-args \
  -p cav_id:=1 \
  -p speed_mps:=0.5 \
  -p max_yaw_rate:=2.0
```

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `cav_id` | 1 | CAV의 차량 ID (1~4) |
| `speed_mps` | 0.5 | 기본 주행 속도 (m/s) |
| `lookahead_m` | 0.3 | Pure Pursuit 선행거리 (m) |
| `max_yaw_rate` | 2.0 | 최대 회전 속도 (rad/s) |

---

## 💻 빌드 및 실행

### 빌드
```bash
cd /root/TEAM_AIM
colcon build --packages-select mission_3
source install/setup.bash
```

### 실행 (7개 터미널)
```bash
# 터미널 1: 시뮬레이터 실행
ros2 launch simulator_launch simulator.launch.py

# 터미널 2: 타워 노드
ros2 run mission_3 control_tower_mission_3

# 터미널 3: 원형 교차로 제어 노드
ros2 run mission_3 control_rotary_mission_3

# 터미널 4~7: CAV_01~04 실행
export CAV_ID=1 && ros2 run mission_3 control_cav_mission_3
export CAV_ID=2 && ros2 run mission_3 control_cav_mission_3
export CAV_ID=3 && ros2 run mission_3 control_cav_mission_3
export CAV_ID=4 && ros2 run mission_3 control_cav_mission_3
```

---

## 📊 제어 흐름도

```
타워 노드:
1. 모든 CAV/HV 위치 모니터링
2. 3개 Zone 각각에서 충돌 위험도 판단
3. Precollision/Imminent Zone 확인
4. Red Flag 또는 Yellow Flag 결정
5. ROI별 진입 우선순위 결정
6. 신호 발행 및 시각화 업데이트

원형 교차로 제어 노드:
1. HV 속도 실시간 측정
2. 호 길이 기반 거리 계산
3. ROI 진입 허가 결정
4. CAV 간 우선순위 조정

CAV 노드:
1. Red/Yellow Flag 수신
2. 경로 추적 (Pure Pursuit)
3. 속도 조정
   ├─ Red Flag = 1: 정지 (0 m/s)
   ├─ Yellow Flag = 1: 50% 감속
   └─ 정상: 기본 속도
4. 명령 발행 (선속도, 각속도)
```

---

## 🔍 주요 함수 (원형 교차로)

### `GetArcLength()`
두 점 사이의 호 길이를 계산합니다 (원형 교차로 전용).

### `measure_hv_velocity()`
HV의 위치 변화로부터 속도를 측정하고, 이동평균으로 노이즈를 제거합니다.

### `calculate_distance()`
두 위치 사이의 직선 거리 (유클리드)를 계산합니다.

### `find_closest_waypoint_index()`
현재 위치에서 경로 상 가장 가까운 웨이포인트를 찾습니다.

---

## ⚙️ 튜닝 가이드

### 차량이 과도하게 정지되는 경우
- **문제**: 충돌 판단 반지름이 너무 큼
- **해결**: `PRECOLLISION_RADIUS` (0.7m) 감소

### 원형 교차로에서 충돌이 발생하는 경우
- **문제**: Yellow Flag 감속이 부족하거나 타이밍 오류
- **해결**: Yellow Flag 감속률 증가 또는 Yellow ROI 반지름 증가

### HV 속도 측정이 부정확한 경우
- **문제**: 이동평균 버퍼 크기가 부적절
- **해결**: `buffer_size` 값 조정 (현재 10개)

### 경로 추적이 진동하는 경우
- **문제**: `max_yaw_rate = 2.0`이 과도함
- **해결**: 값을 1.5 이하로 감소

---

## 📝 핵심 개선 사항 (Mission 1-2 대비)

| 항목 | Mission 1-2 | Mission 3 |
|------|-----------|----------|
| 차량 수 (CAV) | 2대 | 4대 |
| 차량 수 (HV) | 0대 | 2대 |
| 교차로 | 2개 | 3개 + 원형 교차로 |
| max_yaw_rate | 4.0 | **2.0** |
| lookahead_m 게인 | 0.4 | **0.6** |
| Yellow Flag | X | **O** |
| HV 속도 측정 | X | **O** |
| 호 길이 계산 | X | **O** |
| 제어 노드 수 | 2개 | **3개** |

---

## 🎨 RViz 시각화

타워 노드는 다음을 시각화합니다:
- **경로**: CAV별 색상 구분 (파란색, 초록색, 보라색, 주황색)
- **ROI**: 반지름 0.23m 원형 마커
- **3개 Zone**: 경고 영역(0.7m), 긴급 영역(0.5m)
- **현재 위치**: 이동식 마커
- **HV 경로**: 별도 색상

---

## 🐛 문제 해결

| 문제 | 원인 | 해결책 |
|------|------|--------|
| CAV가 움직이지 않음 | 타워 미실행 | 타워 노드 재시작 |
| Red Flag가 계속 유지됨 | 충돌 영역 벗어남 감지 실패 | 타워 로그 확인 |
| HV 속도 측정 오류 | 시간 델타 계산 오류 | 타임스탬프 확인 |
| 경로 파일 오류 | 파일 경로 또는 포맷 오류 | CSV 파일 확인 |
| 원형 교차로에서 충돌 | 호 길이 계산 오류 | 중심 좌표 확인 |

---

## 📚 관련 ROS2 의존성

```xml
<build_depend>rclcpp</build_depend>
<build_depend>geometry_msgs</build_depend>
<build_depend>std_msgs</build_depend>
<build_depend>visualization_msgs</build_depend>
<build_depend>tf2</build_depend>
```

---

## 📄 라이선스 및 저작권

TEAM AIM 프로젝트의 일부입니다.

---

## 👥 기여자

- KAIST 자율주행 팀

---

*Last Updated: 2026-02-02*
