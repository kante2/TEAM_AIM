# TEAM_AIM (KAIST Mobility Challenge)

## 🚀 빠른 시작 - Mission 3 실행 방법

### 📋 Mission 3 구조
```bash
mission_3.sh
├── CAV_ID=32 → control_cav_mission_3 (/CAV_32 발행)
├── CAV_ID=1  → control_cav_mission_3 (/CAV_01 발행)
├── CAV_ID=3  → control_cav_mission_3 (/CAV_03 발행)
├── CAV_ID=6  → control_cav_mission_3 (/CAV_06 발행)
├── CAV_IDS="32,1,3,6" → control_tower_mission_3 (4개 CAV 모두 구독)
└── CAV_IDS="32,1,3,6" → control_rotary_mission_3 (4개 CAV 모두 구독)
```

### 📝 Mission 3 실행 (host => 도커 내부)
```bash
cd /root/TEAM_AIM
chmod +x mission_3.sh
./mission_3.sh
```

**자동 동작:**
- ✅ colcon build로 모든 패키지 컴파일
- ✅ 4개 CAV 프로세스 **병렬 실행** (CAV_ID = 32, 1, 3, 6)
- ✅ Tower 프로세스 **병렬 실행** (CAV_IDS="32,1,3,6" → 4개 CAV 구독)
- ✅ Rotary 프로세스 **병렬 실행** (CAV_IDS="32,1,3,6" → 4개 CAV 구독)

한 줄 명령어 하나로 모든 프로세스가 동시에 시작됩니다!

### 🤖 로봇에서 실행 명령어 (diffrent package -> KAIST_MOBILITY_CHALLENGE_SDK)

**Option 1: mission_3.sh 스크립트 있는 경우**
```bash
# 도커 환경과 동일 (스크립트가 있으면 사용)
./mission_domain_4_SDK.sh # domain connect with docker 
./mission_SDK.sh          # run cmd bridge node
```

**Option 2: 수동 실행 (mission_3.sh가 없거나 조정 필요시)**

각 역할별로 별도 터미널에서 실행:
```bash
# 터미널 1: CAV 1 실행
PROBLEM_ID=4 ROLE=cav CAV_ID=32 ./entrypoint.sh

# 터미널 2: CAV 2 실행
PROBLEM_ID=4 ROLE=cav CAV_ID=1 ./entrypoint.sh

# 터미널 3: CAV 3 실행
PROBLEM_ID=4 ROLE=cav CAV_ID=3 ./entrypoint.sh

# 터미널 4: CAV 4 실행
PROBLEM_ID=4 ROLE=cav CAV_ID=6 ./entrypoint.sh

# 터미널 5: Tower 실행
PROBLEM_ID=4 ROLE=tower CAV_IDS="32,1,3,6" ./entrypoint.sh

# 터미널 6: Rotary 실행
PROBLEM_ID=4 ROLE=rotary CAV_IDS="32,1,3,6" ./entrypoint.sh
```

**환경변수 설명:**
- `PROBLEM_ID=4`: Mission 3 선택
- `ROLE`: 실행 역할 (cav, tower, rotary, simulator)
- `CAV_ID`: CAV 식별자 (각 CAV마다 다른 값)
- `CAV_IDS`: Tower/Rotary가 구독할 CAV ID 목록 (쉼표로 구분)

---

# TEAM_AIM (KAIST Mobility Challenge) ==> simulator
KAIST Mobility Challenge 환경에서 **TEAM_AIM 도커 이미지 빌드 + 컨테이너 실행 + 미션 실행**까지 한 번에 따라 할 수 있도록 정리한 README입니다.  
(대회 측 매뉴얼/구조를 최대한 그대로 사용하는 흐름)

---

## 목차
- [1. 요구 사항](#1-요구-사항)
- [2. Host에서 준비](#2-host에서-준비)
  - [2-1. Docker 설치](#2-1-docker-설치)
  - [2-2. 레포 클론 (final 브랜치)](#2-2-레포-클론-final-브랜치)
  - [2-3. Simulator 설치](#2-3-simulator-설치)
  - [2-4. X11 GUI 설정](#2-4-x11-gui-설정)
  - [2-5. BuildKit 설정](#2-5-buildkit-설정)
- [3. Docker 사용 방법](#3-docker-사용-방법)
  - [3-1. VSCode Dev Containers (권장)](#3-1-vscode-dev-containers-권장)
  - [3-2. CLI로 이미지 빌드](#3-2-cli로-이미지-빌드)
  - [3-3. CLI로 컨테이너 실행](#3-3-cli로-컨테이너-실행)
- [4. Container 내부에서 미션 실행](#4-container-내부에서-미션-실행)
- [5. 미션 스크립트 설명](#5-미션-스크립트-설명)
- [6. 로그 확인](#6-로그-확인)
- [7. 트러블슈팅](#7-트러블슈팅)

---
# 시뮬레이터 사용시, 주의점
L + 1
L + 2
L + 3
L + 4

로 시뮬레이터 상에서 미션별 케이스를 로드한 후, 상황에 따라 cav를 올려놓고,

스페이스바를 두번 빠르게 누른 후, 스페이스바를 한번 더 눌러서 실행한다.
---

## 1. 요구 사항
- OS: Ubuntu (권장)
- 필수 패키지:
  - `git`
  - Docker Engine
  - Docker Compose (plugin 기반 `docker compose` 권장)
- (GPU 사용 시) NVIDIA 드라이버 및 nvidia-container-toolkit 환경이 필요할 수 있습니다.

---

## 2. Host에서 준비

### 2-1. Docker 설치
도커가 없다면 아래 매뉴얼 참고해서 설치하세요.  
- https://github.com/Seo12044/KMC_Docker_Manual

---

### 2-2. 레포 클론 (final 브랜치)
> **중요:** 도커에서 호스트 폴더를 마운트하기 때문에, **호스트에 `~/TEAM_AIM`가 반드시 존재**해야 합니다.

```bash
git clone -b final https://github.com/kante2/TEAM_AIM.git
# 결과: ~/TEAM_AIM


#2-3. Simulator 설치
#TEAM_AIM 폴더 내부에 시뮬레이터를 설치합니다.


cd ~/TEAM_AIM
rm -rf Mobility_Challenge_Simulator
git clone https://github.com/cislab-kaist/Mobility_Challenge_Simulator


##2-4. X11 GUI 설정
##GUI (시뮬레이터/시각화 등)를 위해 X11 권한을 허용합니다 -> IN HOST
xhost +local:root
xhost +local:docker

#2-5. BuildKit 설정
#빌드 성능/호환성 위해 BuildKit을 켭니다.
export DOCKER_BUILDKIT=1
export COMPOSE_DOCKER_CLI_BUILD=1

## 2. how to use docker ===========
# VSCODE 에서 dev containers 를 이용

cd /root/TEAM_AIM/docker_kaist_aim
# build docker image
docker-compose build
# run container
docker run -it --rm \
  --name kaist_aim_container \
  --net=host \
  --privileged \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -v "${HOME}/TEAM_AIM:/root/TEAM_AIM" \
  team_aim

## 3. in docker ==================

## commands using sh
# mission_1_1
chmod +x /root/TEAM_AIM/mission_1_1.sh && ./mission_1_1.sh
```

✅ `mission_1_2` 실행

`mission_1_2.sh`는 `mission_1_2` 실행 스크립트입니다. 실행 권한 부여 후 실행하면 관련 노드가 실행됩니다:

```bash
chmod +x /root/TEAM_AIM/mission_1_2.sh && ./mission_1_2.sh
```

✅ `mission_2` 실행

`mission_2.sh`는 mission_2의 모든 역할을 한 번에 병렬 실행하는 자동화 스크립트입니다.

포함 역할:
- Simulator
- CAV 1~4
- Tower
- Rotary

각 역할은 내부적으로 `entrypoint_ver2.sh`를 호출하여 환경변수에 따라 적절한 노드를 실행합니다. 실행 시 각 역할 로그는 `log/mission_2_*.log`에 저장됩니다.

```bash
chmod +x /root/TEAM_AIM/mission_2.sh && ./mission_2.sh
```

✅ `mission_3` 실행

`mission_3.sh`는 mission_3의 모든 역할을 한 번에 병렬 실행하는 자동화 스크립트입니다.

포함 역할:
- Simulator
- CAV 1~4
- Tower
- Rotary

각 역할은 내부적으로 `entrypoint_ver2.sh`를 호출하여 환경변수에 따라 적절한 노드를 실행합니다. 실행 시 각 역할 로그는 `log/mission_3_*.log`에 저장됩니다.

```bash
chmod +x /root/TEAM_AIM/mission_3.sh && ./mission_3.sh
```

## 6. 로그 확인
미션 2/3은 실행 시 자동으로 로그 파일이 저장됩니다.

Mission 2:

`log/mission_2_*.log`

Mission 3:

`log/mission_3_*.log`

예)

```bash
ls -al log/
tail -n 200 log/mission_2_*.log
```

## 7. 트러블슈팅

### 7-1. GUI 창이 안 뜸 (X11 관련)
호스트에서 아래를 실행했는지 확인:

```bash
xhost +local:root
xhost +local:docker
```

컨테이너 실행 옵션에 아래 항목들이 포함되는지 확인하세요:

- `--env="DISPLAY=$DISPLAY"`
- `--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"`

### 7-2. docker compose가 안 먹음
환경에 따라 `docker-compose` (구버전)만 있을 수 있습니다. 확인:

```bash
docker compose version
docker-compose version
```

### 7-3. 마운트가 안 됨 / 폴더가 비어 보임
호스트에 `~/TEAM_AIM`가 실제로 존재하는지 확인:

```bash
ls -al ~/TEAM_AIM
```

컨테이너 `run` 명령의 `-v` 옵션이 본인 호스트 경로로 맞는지 확인:

```bash
-v ${HOME}/TEAM_AIM:/root/TEAM_AIM
# 또는
-v /home/autonav/TEAM_AIM:/root/TEAM_AIM
```

### 7-4. 실행 권한 에러 (Permission denied)
스크립트에 실행 권한 부여:

```bash
chmod +x /root/TEAM_AIM/mission_1_1.sh
chmod +x /root/TEAM_AIM/mission_1_2.sh
chmod +x /root/TEAM_AIM/mission_2.sh
chmod +x /root/TEAM_AIM/mission_3.sh
```

---

(선택) 빠른 실행 요약 (Host → Docker → Mission)

```bash
# Host
git clone -b final https://github.com/kante2/TEAM_AIM.git
cd ~/TEAM_AIM
rm -rf Mobility_Challenge_Simulator
git clone https://github.com/cislab-kaist/Mobility_Challenge_Simulator

xhost +local:root
xhost +local:docker
export DOCKER_BUILDKIT=1
export COMPOSE_DOCKER_CLI_BUILD=1

# Build
cd ~/TEAM_AIM/docker_kaist_aim
sudo docker compose build

# Run (mount 경로는 본인 환경에 맞게 수정)
docker run -it --rm \
  --name kaist_aim_container \
  --net=host \
  --privileged \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -v ${HOME}/TEAM_AIM:/root/TEAM_AIM \
  team_aim

# In Docker
cd ../TEAM_AIM
chmod +x /root/TEAM_AIM/mission_2.sh && ./mission_2.sh
```
