# TEAM_AIM (KAIST Mobility Challenge)

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


3. Docker 사용 방법
3-1. VSCode Dev Containers (권장)

VSCode에서 Dev Containers를 이용해 컨테이너로 바로 진입하여 개발/실행할 수 있습니다.



3-2. CLI로 이미지 빌드
도커 관련 폴더로 이동 후 이미지 빌드:
cd ~/TEAM_AIM/docker_kaist_aim
sudo docker compose build


3-3. CLI로 컨테이너 실행
아래는 실행 커맨드입니다.
docker run -it --rm \
  --name kaist_aim_container \
  --net=host \
  --privileged \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -v /home/autonav/TEAM_AIM:/root/TEAM_AIM \
  team_aim
마운트 경로 주의
위 커맨드의 마운트 옵션은 호스트 경로가 /home/autonav/TEAM_AIM 인 케이스입니다.
본인 계정이 다른 경우 아래처럼 바꿔서 실행하세요.

/

4. Container 내부에서 미션 실행
컨테이너 내부에서 TEAM_AIM 폴더로 이동합니다.
(컨테이너 내부 경로가 kaist_contest_ws 등인 경우를 포함하여, 최종적으로 TEAM_AIM로 이동)

cd ..
cd TEAM_AIM


/
5. 미션 스크립트 설명
# TEAM_AIM (KAIST Mobility Challenge)

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
도커가 없다면 아래 매뉴얼 참고해서 설치하세요:

https://github.com/Seo12044/KMC_Docker_Manual

### 2-2. 레포 클론 (final 브랜치)
**중요:** 도커에서 호스트 폴더를 마운트하기 때문에, **호스트에 `~/TEAM_AIM`가 반드시 존재**해야 합니다.

```bash
git clone -b final https://github.com/kante2/TEAM_AIM.git
# 결과: ~/TEAM_AIM
```

### 2-3. Simulator 설치
TEAM_AIM 폴더 내부에 시뮬레이터를 설치합니다:

```bash
cd ~/TEAM_AIM
rm -rf Mobility_Challenge_Simulator
git clone https://github.com/cislab-kaist/Mobility_Challenge_Simulator
```

### 2-4. X11 GUI 설정
GUI (시뮬레이터/시각화 등)를 위해 X11 권한을 허용합니다 (호스트에서 실행):

```bash
xhost +local:root
xhost +local:docker
```

### 2-5. BuildKit 설정
빌드 성능/호환성 위해 BuildKit을 켭니다:

```bash
export DOCKER_BUILDKIT=1
export COMPOSE_DOCKER_CLI_BUILD=1
```

## 3. Docker 사용 방법

### 3-1. VSCode Dev Containers (권장)
VSCode에서 Dev Containers를 이용해 컨테이너로 바로 진입하여 개발/실행할 수 있습니다.

### 3-2. CLI로 이미지 빌드
도커 관련 폴더로 이동 후 이미지 빌드:

```bash
cd ~/TEAM_AIM/docker_kaist_aim
sudo docker compose build
```

### 3-3. CLI로 컨테이너 실행
아래는 실행 예시 커맨드입니다. 마운트 경로는 본인 호스트 경로로 변경하세요:

```bash
docker run -it --rm \
  --name kaist_aim_container \
  --net=host \
  --privileged \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  -v "${HOME}/TEAM_AIM:/root/TEAM_AIM" \
  team_aim
```


## 4. Container 내부에서 미션 실행
컨테이너 내부에서 `TEAM_AIM` 폴더로 이동합니다 (컨테이너 내부 경로가 `kaist_contest_ws` 등인 경우 포함):

```bash
cd ..
cd TEAM_AIM
```

## 5. 미션 스크립트 설명

✅ `mission_1_1` 실행

`mission_1_1.sh`는 `mission_1_1` 실행 스크립트입니다. 실행 권한 부여 후 실행하면 관련 노드가 실행됩니다:

```bash
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
