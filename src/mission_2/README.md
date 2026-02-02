# Mission 2: 차선 변경 기반 협력 주행 (Lane-Change Cooperative Driving)

## 📋 개요

**Mission 2**는 자동주행 차량(CAV 1대)이 **3개 차선 도로**에서 **수동조종 차량(HV 2대)과 협력**하여 안전하게 주행하고 목적지에 도달하는 미션입니다. 

이 미션의 핵심은:
- **다중 차선 관리**: 3개 차선(좌측, 중앙, 우측) 경로 사전 로딩
- **동적 차선 변경**: 상황에 따라 좌/중/우 차선으로 동적 전환
- **HV 협력 제어**: 2대의 HV(HV20, HV24) 속도를 실시간 측정하여 협력
- **복수 충돌 방지 영역(Zone)**: 5개의 Zone을 통한 세밀한 충돌 회피
- **선행선(Solid Line) 감지**: 차선 변경 금지 구간 자동 감지

---

## 🎯 미션 목표

- **안전한 차선 변경**: HV와의 충돌 없이 차선 변경 수행
- **HV 협력**: 2대의 HV 속도를 실시간 측정하고 이에 맞춰 주행
- **다중 Zone 충돌 회피**: 5개 Zone에서의 동적 충돌 방지
- **경로 추적**: 선택된 차선의 경로를 정확하게 추적
- **목적지 도달**: 정상적으로 주행 완료

---

## 🏗️ 주요 구조

### 📁 파일 구성
```
mission_2/
├── CMakeLists.txt           # CMake 빌드 설정
├── package.xml              # ROS2 패키지 메타데이터
├── launch/                  # ROS2 실행 설정
└── src/
    └── control_cav_mission_2.cpp  # 차선 변경 제어 및 협력 주행
```

### 📦 ROS2 노드

#### `control_cav_mission_2` 노드
단일 CAV의 차선 변경 결정, HV 협력, 경로 추적을 모두 담당하는 통합 제어 노드입니다.

---

## 🔧 핵심 기능

### 1️⃣ **3개 차선 경로 관리**

```cpp
// 3개 차선의 경로 데이터 구조
static std::vector<PathPoint> lane_paths[4];  // [1]=LEFT, [2]=CENTER, [3]=RIGHT
static int current_lane = 2;  // 초기값: 중앙 차선

struct PathPoint {
    double x;
    double y;
};
```

**경로 로딩:**
- `path_mission2_lane1.csv`: 좌측 차선
- `path_mission2_lane2.csv`: 중앙 차선 (기본)
- `path_mission2_lane3.csv`: 우측 차선

각 차선은 독립적인 웨이포인트 시퀀스로 관리됩니다.

### 2️⃣ **Pure Pursuit 경로 추적 (차선별)**

**파라미터:**
- **`speed_mps`**: 기본 속도 (0.5 m/s)
- **`lookahead_m`**: 선행거리 (0.3 m)
- **`max_yaw_rate`**: 최대 회전 속도 (1.5 rad/s) ← **보수적 설정**

현재 선택된 차선(`current_lane`)의 경로를 따라 Pure Pursuit 제어를 수행합니다:

```cpp
integrate_path_vector = lane_paths[current_lane];  // 현재 차선 경로 활성화
```

### 3️⃣ **HV 속도 실시간 측정**

2대의 HV(HV20, HV24)의 속도를 실시간으로 측정합니다:

```cpp
struct HVState {
    int id;
    bool is_first_msg = true;
    double prev_x = 0.0, prev_y = 0.0;
    rclcpp::Time last_time;
    
    // 이동 평균 필터 (노이즈 제거)
    std::deque<double> vel_buffer;
    const size_t buffer_size = 10;  // 최근 10개 속도의 이동평균
};

static double measured_hv20_vel = 1.3;  // Lane 3 (HV20)
static double measured_hv24_vel = 1.3;  // Lane 2 (HV24)
```

**측정 방식:**
1. 이전 위치와의 거리 계산
2. 시간 차이로 속도 계산
3. 이동평균으로 노이즈 제거
4. 상한선(2.5 m/s) 처리로 이상 값 필터링

### 4️⃣ **5개 Zone을 통한 충돌 방지**

미션 2에서는 5개의 서로 다른 충돌 방지 영역이 정의됩니다:

#### **Zone 1: CAV와 HV의 초기 만남**
```cpp
// HV Zone 1: 다각형 영역 (14개 점으로 정의)
static std::vector<PathPoint> hv_zone_polygon = {...};

// CAV Zone 1: 선분 (4.5, -2.55) ~ (3.0, -2.55)
static PathPoint cav_zone_start = {4.5, -2.549999952316284};
static PathPoint cav_zone_end   = {3.0, -2.549999952316284};

// 충돌 검사: HV가 HV Zone 내 AND CAV가 CAV Zone 내 → 충돌!
bool check_zone_collision();
```

#### **Zone 2: HV와 CAV의 통과**
```cpp
// CAV Zone 2: 수직 선분 (5.058, 1.02) ~ (5.058, 0.0)
static PathPoint cav_zone2_start = {5.058333396911621, 1.0199999809265137};
static PathPoint cav_zone2_end = {5.058333396911621, 0.0};

// HV Zone 2: 수직 선분 (5.308, 0.4) ~ (5.308, -0.3)
static PathPoint hv_zone2_start = {5.308333396911621, 0.4};
static PathPoint hv_zone2_end = {5.308333396911621, -0.3};

bool check_zone2_collision();
```

#### **Zone 3, 3-1, 3-2: 차선 변경 구간 (실선)**
```cpp
// Zone 3: 수평 선분 (3.6, 2.55) ~ (4.413, 2.55)
// Zone 3-1: 수평 선분 (2.683, 2.308) ~ (4.413, 2.308)
// Zone 3-2: 수평 선분 (2.0, 2.55) ~ (3.6, 2.55)

// 이들 Zone 내에서는 차선 변경 금지 (실선 구간)
```

#### **Zone 4, 5: 추가 안전 영역**
```cpp
// Zone 4: 수직 선분 (5.058, 2.308) ~ (5.058, 1.74)
// Zone 5: 수직 선분 (5.292, 2.308) ~ (5.292, -0.27)
```

**Zone 검사 방식:**
- **선분 Zone**: CAV/HV의 위치에서 선분까지의 최단거리 ≤ 0.1m 판정
- **다각형 Zone**: 각 점으로부터의 거리 ≤ 0.1m 판정

### 5️⃣ **차선 겹침(Overlap) 감지**

Lane 2(중앙)와 Lane 3(우측)이 겹치는 구간을 사전에 계산하여 저장:

```cpp
// 초기화 시 겹침 구간 계산
void init_overlap_region();

// 출력 예시:
// overlap_start_idx = 50  (Lane 3 기준 인덱스)
// overlap_end_idx = 120   (Lane 3 기준 인덱스)

// 겹침 구간 내에서는 차선 변경 전략이 변경될 수 있음
```

### 6️⃣ **Red Flag 대응**

타워로부터의 정지 신호에 즉시 반응:

```
Red Flag = 1: 모든 명령 정지 (선속도 = 0)
Red Flag = 0: 정상 차선 변경 및 경로 추적 로직 실행
```

---

## 📡 ROS2 토픽

### 구독(Subscription)
| 토픽명 | 메시지 타입 | 설명 |
|--------|-----------|------|
| `/CAV_01` | `geometry_msgs/msg/PoseStamped` | CAV의 위치/자세 정보 |
| `/HV_20`, `/HV_24` | `geometry_msgs/msg/PoseStamped` | HV의 위치/자세 (속도 측정용) |
| `/CAV_01_RED_FLAG` | `std_msgs/msg/Int32` | 타워의 정지 신호 |

### 발행(Publication)
| 토픽명 | 메시지 타입 | 설명 |
|--------|-----------|------|
| `/CAV_01_accel` | `geometry_msgs/msg/Accel` | 계산된 선속도 및 각속도 명령 |

---

## 🚀 파라미터

노드 실행 시 아래 파라미터로 제어 성능을 튜닝할 수 있습니다:

```bash
ros2 run mission_2 control_cav_mission_2 \
  --ros-args \
  -p speed_mps:=0.5 \
  -p lookahead_m:=0.3 \
  -p max_yaw_rate:=1.5
```

| 파라미터 | 기본값 | 범위 | 설명 |
|---------|--------|------|------|
| `speed_mps` | 0.5 | 0.1~2.0 | 기본 주행 속도 (m/s) |
| `lookahead_m` | 0.3 | 0.1~0.8 | Pure Pursuit 선행거리 (m) |
| `max_yaw_rate` | 1.5 | 0.5~3.0 | 최대 회전 속도 제한 (rad/s) |
| `cav_id` | 1 | 1 | CAV의 차량 ID (고정) |

---

## 💻 빌드 및 실행

### 빌드
```bash
cd /root/TEAM_AIM
colcon build --packages-select mission_2
source install/setup.bash
```

### 실행 (시뮬레이터와 함께)
```bash
# 터미널 1: 시뮬레이터 실행
ros2 launch simulator_launch simulator.launch.py

# 터미널 2: Mission 2 노드 실행
export CAV_ID=1
ros2 run mission_2 control_cav_mission_2
```

---

## 📊 제어 흐름도

```
1. 초기화
   ├─ 3개 차선 경로 CSV 로딩
   ├─ 차선 겹침 영역 계산 (Zone 3,3_1,3_2 등)
   ├─ 5개 Zone 정의
   └─ ROS2 Pub/Sub 설정 (CAV, HV20, HV24)

2. 메인 루프 (100 Hz)
   ├─ CAV 위치/자세 수신
   ├─ HV20, HV24 위치 수신 및 속도 측정
   ├─ 5개 Zone 충돌 검사
   │  ├─ Zone 1: 초기 만남 충돌 검사
   │  ├─ Zone 2: 통과 충돌 검사
   │  └─ Zone 3~5: 차선 변경 금지 구간 감지
   ├─ 차선 변경 결정
   │  ├─ 충돌 위험 없는지 확인
   │  ├─ 실선 구간(Zone 3 등)인지 확인
   │  └─ 선택할 차선 결정 (1=좌, 2=중앙, 3=우)
   ├─ 현재 차선 경로로 Pure Pursuit 계산
   ├─ Red Flag 확인 후 명령 생성
   └─ 속도 및 회전 명령 발행

3. 종료
   ├─ 목적지 도달 감지
   └─ 정지 명령 유지
```

---

## 🔍 주요 함수 설명

### Zone 검사 함수들

**`is_cav_in_zone*()`**
- CAV가 특정 Zone 내에 있는지 선분으로부터 거리 계산으로 판정
- 검사 반지름: 0.1m

**`is_hv_in_zone*()`**
- HV가 특정 Zone 내에 있는지 판정
- 선분 또는 다각형 Zone 지원

**`check_zone_collision()`, `check_zone2_collision()`, ...**
- 특정 Zone에서 CAV와 HV가 동시에 있는지 확인
- 충돌 위험 판정

### HV 속도 측정

**`MeasureHVVelocity()`**
- HV의 위치 변화로부터 속도 측정
- 이동평균 필터 적용으로 노이즈 제거
- 측정된 속도는 `measured_hv20_vel`, `measured_hv24_vel`에 저장

### 경로 관리

**`get_lane_start_idx()`**
- 현재 위치에서 특정 차선의 가장 가까운 웨이포인트 인덱스 반환

**`init_overlap_region()`**
- Lane 2와 Lane 3이 겹치는 구간(인덱스 범위) 사전 계산

---

## ⚙️ 튜닝 가이드

### HV 속도 측정이 부정확한 경우
- **문제**: 이동평균 버퍼 크기가 부적절
- **해결**: `buffer_size` 값 조정 (현재 10개)
  ```cpp
  const size_t buffer_size = 10;  // 5~20 범위에서 튜닝
  ```

### 차선 변경이 과도하게 자주 발생하는 경우
- **문제**: Zone 충돌 판정 반지름이 너무 작음
- **해결**: Zone 검사 반지름 증가
  ```cpp
  const double detection_radius = 0.1;  // 0.15~0.2로 증가
  ```

### 차선 변경이 무시 되는 경우
- **문제**: 실선 구간(Zone 3~5) 감지 로직이 과도함
- **해결**: 실선 구간 거리 조정 또는 로직 검토

### 경로 추적이 진동하는 경우
- **문제**: `max_yaw_rate = 1.5`가 적절하지 않음
- **해결**: 값을 1.0~2.0 범위로 조정

---

## 📝 핵심 구조 및 변수

### 전역 변수 요약
```cpp
// 경로 데이터
std::vector<PathPoint> lane_paths[4];        // 3개 차선 경로
std::vector<PathPoint> integrate_path_vector; // 현재 활성화 경로
int current_lane = 2;                        // 현재 차선 (1,2,3)

// HV 위치 및 속도
std::map<int, std::pair<double, double>> hv_positions;  // HV ID -> (x,y)
double measured_hv20_vel = 1.3;             // HV20 속도
double measured_hv24_vel = 1.3;             // HV24 속도

// 충돌 방지 Zone 플래그들
bool zone_collision_flag = false;           // Zone 1 충돌
bool zone2_collision_flag = false;          // Zone 2 충돌
bool zone3_collision_flag = false;          // Zone 3 (실선)
// ... 및 기타 Zone 플래그들

// 차선 겹침 영역
int overlap_start_idx = -1;                 // Lane 3 기준 시작 인덱스
int overlap_end_idx = -1;                   // Lane 3 기준 종료 인덱스
```

---

## 🎨 시각화 및 로깅

코드 내 디버깅 출력:
```cpp
cout << "[ZONE1_DETECTION] CAV in CAV Zone1!" << std::endl;
cout << "[ZONE2_COLLISION] HV_" << hv_id << " in HV Zone2 + CAV in CAV Zone2!" << std::endl;
cout << "[ZONE3_DETECTION] CAV in CAV Zone3!" << std::endl;
```

이를 통해 각 Zone 진입을 실시간으로 모니터링할 수 있습니다.

---

## 🐛 문제 해결

| 문제 | 원인 | 해결책 |
|------|------|--------|
| CAV가 차선을 변경하지 않음 | 모든 Zone에서 충돌 감지 | Zone 반지름 검토 |
| HV 속도가 0으로 계속 나옴 | HV 토픽 구독 실패 | `ros2 topic list` 확인 |
| 경로 파일 로드 실패 | 3개 차선 CSV 파일 누락 | 경로 파일 확인 |
| 차선 겹침 계산 오류 | Lane 경로 데이터 불일치 | CSV 파일 검증 |
| 과도한 회전 | `max_yaw_rate` 과대 | 값 감소 (1.5 권장) |

---

## 📚 관련 ROS2 의존성

```xml
<build_depend>rclcpp</build_depend>
<build_depend>geometry_msgs</build_depend>
<build_depend>std_msgs</build_depend>
<build_depend>tf2</build_depend>
```

---

## 📊 Mission 2 특화 기능 요약

| 기능 | 구현 | 설명 |
|------|------|------|
| 다중 경로 관리 | 3개 차선 별도 로드 | 동적 차선 변경 지원 |
| HV 협력 제어 | 속도 실시간 측정 | 2대 HV와 동기화 |
| 복수 Zone | 5개 Zone 정의 | 세밀한 충돌 회피 |
| 실선 감지 | Zone 3,3-1,3-2 | 차선 변경 금지 구간 |
| 겹침 영역 계산 | `init_overlap_region()` | 차선 오버래핑 구간 파악 |
| 이동평균 필터 | 10개 샘플 버퍼 | HV 속도 노이즈 제거 |

---

## 📄 라이선스 및 저작권

TEAM AIM 프로젝트의 일부입니다.

---

## 👥 기여자

- KAIST 자율주행 팀

---

*Last Updated: 2026-02-02*
