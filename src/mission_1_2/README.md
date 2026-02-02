# Mission 1-2: 차량 간 협력 주행 (Multi-Vehicle Cooperative Trajectory Tracking)

## 📋 개요

**Mission 1-2**는 두 개의 자동주행 차량(CAV1, CAV2)이 **협력주행(Cooperative Driving)**을 수행하는 미션입니다. 타워 노드에서 관리하는 **ROI(Region of Interest)** 및 **충돌 방지 영역(Precollision/Imminent Zones)**을 통해 두 차량이 안전하게 교차로를 통과하고 합류 차선에 진입하도록 제어됩니다.

---

## 🎯 미션 목표

- **경로 추적**: 각 CAV가 자신의 고유 경로(mission1_01.csv / mission1_02.csv)를 따라 주행
- **충돌 회피**: 두 차량이 교차할 때 ROI 진입 순서를 조정하여 충돌 방지
- **협력 제어**: 타워의 신호(Red Flag, 목표 속도)에 따라 속도 조정
- **합류 차선 통과**: 최종 ROI-4(합류 지점)를 안전하게 통과

---

## 🏗️ 주요 구조

### 📁 파일 구성
```
mission_1_2/
├── CMakeLists.txt           # CMake 빌드 설정
├── package.xml              # ROS2 패키지 메타데이터
└── src/
    ├── control_cav_mission_1_2.cpp      # CAV 제어 노드
    └── control_tower_mission_1_2.cpp    # 타워(제어 센터) 노드
```

### 📦 ROS2 노드

#### 1. `control_cav_mission_1_2` 노드
각 자동주행 차량의 경로 추적 및 속도 제어를 담당합니다.

#### 2. `control_tower_mission_1_2` 노드
모든 차량의 위치를 모니터링하고, ROI 진입 우선순위를 결정하며, 충돌 회피 신호를 발행합니다.

---

## 🔧 핵심 기능

### 1️⃣ **경로 로딩 및 CAV별 할당**

각 CAV는 자신의 ID에 따라 다른 경로 파일을 로드합니다:

```cpp
if (my_id_str == "01") {
    path_with_id_csv = "/root/TEAM_AIM/src/global_path/path_mission1_01.csv";
} else if (my_id_str == "02") {
    path_with_id_csv = "/root/TEAM_AIM/src/global_path/path_mission1_02.csv";
}
```

| CAV ID | 경로 파일 |
|--------|----------|
| CAV_01 | `path_mission1_01.csv` |
| CAV_02 | `path_mission1_02.csv` |

### 2️⃣ **Pure Pursuit 경로 추적**

Mission 1-1과 유사하지만, **동적 선행거리 계산**이 최적화되었습니다:

**파라미터:**
- **`speed_mps`**: 기본 속도 (0.5 m/s)
- **`lookahead_m`**: 선행거리 (0.3 m)
- **`max_yaw_rate`**: 최대 회전 속도 (4.0 rad/s) ← **Mission 1-1의 5.5에서 개선**

**동적 선행거리 계산:**
```cpp
void GetLd(ControllerState& st) {
  double gain_ld = 0.4;
  double max_ld  = 0.355;
  double min_ld  = 0.15;
  double ld = gain_ld * velocity;
  st.lookahead_m = std::max(min_ld, std::min(max_ld, ld));
}
```

### 3️⃣ **두 차량의 경로**

```
CAV_01 경로: 위쪽 차선 → ROI-1 → ROI-2 → ROI-4(합류)
CAV_02 경로: 아래쪽 차선 → ROI-3 → ROI-4(합류)
```

### 4️⃣ **ROI(Region of Interest) 정의**

두 차량이 통과해야 하는 중요 영역들:

```cpp
std::map<int, Pose> rois = {
  {1, {1.1329693794250488, -0.5619539022445679}},  // CAV1이 먼저 지나는 ROI
  {2, {2.125271797180176, -0.6247305870056152}},   // CAV1이 두 번째로 지나는 ROI
  {3, {1.774999976158142, -1.8}},                  // CAV2가 지나는 ROI
  {4, {5.05833333333333, 0.666666666666667}}       // 합류 차선 ROI
};

const double ROI_RADIUS = 0.23;         // ROI 1, 2, 3 반지름
const double ROI_MERGE_RADIUS = 0.6;    // ROI 4 반지름 (합류 구간용, 더 크게)
```

### 5️⃣ **충돌 방지 영역 (Collision Zones)**

타워에서 두 차량 사이의 거리를 모니터링하고 충돌 위험을 판단합니다:

```cpp
// Zone 정의 (교차로 중심점)
std::map<int, Pose> zones = {
  {2, {-2.333333333, 0.0}},    // 교차로 2
  {4, {2.001666784286499, 2.7}} // 교차로 4 (합류)
};

// Precollision Zone: 반지름 0.7m (경고)
bool is_in_precollision_zone(Pose cav_pose, Pose zone_origin, double radius)

// Imminent Collision Zone: 반지름 0.5m (긴급 정지)
bool is_in_imminent_collision_zone(Pose cav_pose, Pose zone_origin, double radius)
```

### 6️⃣ **타워의 충돌 판단 및 신호**

타워 노드에서 두 CAV가 동시에 같은 ROI에 진입할 위험을 감지하고:

1. **Red Flag (정지 신호)** 발행: 한 대 정지
2. **목표 속도 명령**: 우선순위에 따라 속도 조정
3. **시각화**: RViz에서 ROI와 경로 시각화

```cpp
// 타워가 발행하는 신호
/CAV_01_RED_FLAG (Int32)      // Red Flag = 1 (정지), 0 (진행)
/CAV_02_RED_FLAG (Int32)
/CAV_01_target_vel (Float64)  // 목표 속도 명령
/CAV_02_target_vel (Float64)
```

---

## 📡 ROS2 토픽

### CAV 노드 (구독)
| 토픽명 | 메시지 타입 | 설명 |
|--------|-----------|------|
| `/CAV_01` ~ `/CAV_02` | `geometry_msgs/msg/PoseStamped` | 각 CAV의 위치/자세 |
| `/CAV_01_RED_FLAG`, `/CAV_02_RED_FLAG` | `std_msgs/msg/Int32` | 타워의 정지 신호 |
| `/CAV_01_target_vel`, `/CAV_02_target_vel` | `std_msgs/msg/Float64` | 타워의 목표 속도 |

### CAV 노드 (발행)
| 토픽명 | 메시지 타입 | 설명 |
|--------|-----------|------|
| `/CAV_01_accel`, `/CAV_02_accel` | `geometry_msgs/msg/Accel` | 계산된 명령 (선속도, 각속도) |

### 타워 노드 (구독)
| 토픽명 | 메시지 타입 | 설명 |
|--------|-----------|------|
| `/CAV_01` ~ `/CAV_02` | `geometry_msgs/msg/PoseStamped` | 모든 CAV 위치 모니터링 |

### 타워 노드 (발행)
| 토픽명 | 메시지 타입 | 설명 |
|--------|-----------|------|
| `/CAV_01_RED_FLAG`, `/CAV_02_RED_FLAG` | `std_msgs/msg/Int32` | 정지 신호 (1=정지, 0=진행) |
| `/CAV_01_target_vel`, `/CAV_02_target_vel` | `std_msgs/msg/Float64` | 목표 속도 (m/s) |
| `/visualization_marker_array` | `visualization_msgs/msg/MarkerArray` | RViz 시각화 (경로, ROI) |

---

## 🚀 파라미터

### CAV 노드 파라미터
```bash
ros2 run mission_1_2 control_cav_mission_1_2 \
  --ros-args \
  -p cav_id:=1 \
  -p speed_mps:=2.0 \
  -p lookahead_m:=0.4 \
  -p max_yaw_rate:=4.0
```

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `cav_id` | 1 | CAV의 차량 ID (1 또는 2) |
| `speed_mps` | 0.5 | 기본 주행 속도 (m/s) |
| `lookahead_m` | 0.4 | Pure Pursuit 선행거리 (m) |
| `max_yaw_rate` | 4.0 | 최대 회전 속도 (rad/s) |

### 타워 노드 파라미터
```bash
ros2 run mission_1_2 control_tower_mission_1_2 \
  --ros-args \
  -p collision_distance_threshold:=0.5
```

---

## 💻 빌드 및 실행

### 빌드
```bash
cd /root/TEAM_AIM
colcon build --packages-select mission_1_2
source install/setup.bash
```

### 실행 (3개 터미널)
```bash
# 터미널 1: 시뮬레이터 실행
ros2 launch simulator_launch simulator.launch.py

# 터미널 2: 타워 노드 실행
ros2 run mission_1_2 control_tower_mission_1_2

# 터미널 3: CAV_01 실행
export CAV_ID=1
ros2 run mission_1_2 control_cav_mission_1_2

# 터미널 4: CAV_02 실행
export CAV_ID=2
ros2 run mission_1_2 control_cav_mission_1_2
```

---

## 📊 제어 흐름도

```
타워 노드:
1. 모든 CAV 위치 모니터링
2. 각 CAV의 ROI 진입 상태 확인
3. 충돌 가능성 판단
   ├─ Precollision Zone: 경고
   └─ Imminent Collision Zone: Red Flag = 1 (정지)
4. 우선순위 결정 및 신호 발행
5. RViz 시각화 업데이트

CAV 노드:
1. 타워의 Red Flag 수신
2. 타워의 목표 속도 수신
3. 자신의 위치 기반 경로 추적
4. Red Flag = 1 → 즉시 정지 (linear_x = 0)
5. Red Flag = 0 → Pure Pursuit 제어 실행
6. 명령 발행 (선속도, 각속도)
```

---

## 🔍 주요 함수 (타워)

### `calculate_distance()`
두 위치 사이의 유클리드 거리를 계산합니다.

### `is_in_precollision_zone()`
CAV가 위험 경고 영역(0.7m)에 있는지 확인합니다.

### `is_in_imminent_collision_zone()`
CAV가 긴급 정지 영역(0.5m)에 있는지 확인합니다.

### `load_csv_file()`
경로 데이터를 CSV 파일로부터 로드합니다.

---

## ⚙️ 튜닝 가이드

### 충돌이 자주 발생하는 경우
- **문제**: 타워의 충돌 판단 반지름이 너무 작음
- **해결**: `PRECOLLISION_RADIUS`와 `IMMINENT_RADIUS` 값 증가

### 한 차량이 과도하게 정지되는 경우
- **문제**: Red Flag 타임아웃 설정 부재
- **해결**: 타워 코드에서 시간 기반 플래그 해제 로직 추가

### 경로 추적이 부정확한 경우
- **문제**: CAV별 경로 파일 경로 오류
- **해결**: `/root/TEAM_AIM/src/global_path/` 폴더 확인

---

## 📝 핵심 개선 사항 (Mission 1-1 대비)

| 항목 | Mission 1-1 | Mission 1-2 |
|------|-----------|-----------|
| 차량 수 | 1대 | 2대 |
| 경로 | 고정 (01만) | 동적 (01, 02) |
| 제어 | 독립 주행 | 협력 주행 |
| max_yaw_rate | 5.5 | **4.0** |
| 타워 모드 | X | O (목표 속도 제어) |
| 충돌 회피 | X | O (Red Flag) |
| 우선순위 | 없음 | ROI별 결정 |

---

## 🎨 RViz 시각화

타워 노드는 다음을 RViz에서 시각화합니다:
- **경로**: CAV별 색상 구분 (파란색=CAV1, 초록색=CAV2)
- **ROI**: 반지름 0.23m 원형 마커
- **충돌 영역**: 반지름 0.7m (경고), 0.5m (긴급) 원형 마커
- **현재 위치**: 이동식 마커

---

## 🐛 문제 해결

| 문제 | 원인 | 해결책 |
|------|------|--------|
| CAV가 움직이지 않음 | 타워 미실행 또는 Red Flag 지속 | 타워 노드 실행 여부 확인 |
| Red Flag가 해제 안됨 | 타워 로직 오류 | 타워 노드 로그 확인 |
| 충돌 발생 | 충돌 판단 반지름 부족 | 반지름 값 증가 |
| 경로 파일 오류 | 파일 경로 오류 또는 포맷 오류 | CSV 파일 확인 |

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
