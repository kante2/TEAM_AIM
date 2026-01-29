# TEAM AIM - Mission Simulator Setup Guide

## Overview
모든 미션은 도메인 ID 100으로 설정되어 있습니다.  
각 미션별 shell script를 실행하면 시뮬레이터부터 도메인까지 모든 것이 자동으로 설정됩니다.

## Mission 실행 방법

### Mission 1-1
```bash
./mission_1_1.sh
```

### Mission 1-2
```bash
./mission_1_2.sh
```

### Mission 2
```bash
./mission_2.sh
```

### Mission 3
```bash
./mission_3.sh
```

### Mission Domain 1
```bash
./mission_domain_1.sh
```

### Mission Domain 2
```bash
./mission_domain_2.sh
```

### Mission Domain 3
```bash
./mission_domain_3.sh
```

### Mission Domain 4
```bash
./mission_domain_4.sh
```

---

## 호스트에서 ROS2 토픽 확인 방법

### 1단계: 환경 변수 설정

호스트 터미널에서 다음 환경 변수를 설정합니다:

```bash
# ROS Domain ID 설정 (도메인 ID 100 사용)
# 실제 로봇: 100 | 시뮬레이터: 102
export ROS_DOMAIN_ID=100

# Localhost 통신 비활성화 (다른 호스트와 통신하기 위함)
export ROS_LOCALHOST_ONLY=0

# RMW (ROS Middleware) 구현 설정
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### 2단계: 토픽 확인

```bash
ros2 topic list
```

---

## 각 미션별 토픽 확인 체크리스트

### Mission 1-1
1. 터미널 1에서 실행:
   ```bash
   ./mission_1_1.sh
   ```

2. 새 터미널(터미널 2)을 열어 환경 변수 설정:
   ```bash
   export ROS_DOMAIN_ID=100
   export ROS_LOCALHOST_ONLY=0
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   ```

3. 토픽 확인:
   ```bash
   ros2 topic list
   ```

---

### Mission 1-2
1. 터미널 1에서 실행:
   ```bash
   ./mission_1_2.sh
   ```

2. 새 터미널(터미널 2)을 열어 환경 변수 설정:
   ```bash
   export ROS_DOMAIN_ID=100
   export ROS_LOCALHOST_ONLY=0
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   ```

3. 토픽 확인:
   ```bash
   ros2 topic list
   ```

---

### Mission 2
1. 터미널 1에서 실행:
   ```bash
   ./mission_2.sh
   ```

2. 새 터미널(터미널 2)을 열어 환경 변수 설정:
   ```bash
   export ROS_DOMAIN_ID=100
   export ROS_LOCALHOST_ONLY=0
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   ```

3. 토픽 확인:
   ```bash
   ros2 topic list
   ```

---

### Mission 3
1. 터미널 1에서 실행:
   ```bash
   ./mission_3.sh
   ```

2. 새 터미널(터미널 2)을 열어 환경 변수 설정:
   ```bash
   export ROS_DOMAIN_ID=100
   export ROS_LOCALHOST_ONLY=0
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   ```

3. 토픽 확인:
   ```bash
   ros2 topic list
   ```

---

### Mission Domain 1
1. 터미널 1에서 실행:
   ```bash
   ./mission_domain_1.sh
   ```

2. 새 터미널(터미널 2)을 열어 환경 변수 설정:
   ```bash
   export ROS_DOMAIN_ID=100
   export ROS_LOCALHOST_ONLY=0
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   ```

3. 토픽 확인:
   ```bash
   ros2 topic list
   ```

---

### Mission Domain 2
1. 터미널 1에서 실행:
   ```bash
   ./mission_domain_2.sh
   ```

2. 새 터미널(터미널 2)을 열어 환경 변수 설정:
   ```bash
   export ROS_DOMAIN_ID=100
   export ROS_LOCALHOST_ONLY=0
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   ```

3. 토픽 확인:
   ```bash
   ros2 topic list
   ```

---

### Mission Domain 3
1. 터미널 1에서 실행:
   ```bash
   ./mission_domain_3.sh
   ```

2. 새 터미널(터미널 2)을 열어 환경 변수 설정:
   ```bash
   export ROS_DOMAIN_ID=100
   export ROS_LOCALHOST_ONLY=0
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   ```

3. 토픽 확인:
   ```bash
   ros2 topic list
   ```

---

### Mission Domain 4
1. 터미널 1에서 실행:
   ```bash
   ./mission_domain_4.sh
   ```

2. 새 터미널(터미널 2)을 열어 환경 변수 설정:
   ```bash
   export ROS_DOMAIN_ID=100
   export ROS_LOCALHOST_ONLY=0
   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   ```

3. 토픽 확인:
   ```bash
   ros2 topic list
   ```

---

## 주의사항

- **Domain ID**: 모든 미션은 도메인 ID 100을 사용합니다
- **실제 로봇 vs 시뮬레이터**: 
  - 실제 로봇: `ROS_DOMAIN_ID=100`
  - 시뮬레이터: `ROS_DOMAIN_ID=102` (필요시)
- **환경 변수는 새 터미널을 열 때마다 다시 설정해야 합니다**
- 호스트에서 토픽을 확인하려면 반드시 위의 환경 변수 3가지를 설정해야 합니다
