# COMMANDS  TEAM :AIM ==============

## 1. in host =================
git clone -b final https://github.com/kante2/TEAM_AIM.git
--> 마운트를 시켜놓았기 떄문에 호스트에 TEAM_AIM 이 존재해야 한다. (~/aim_ws)

xhost +local:root

## 2. how to use docker ===========
cd /root/TEAM_AIM/docker_kaist_aim
# build docker image
docker-compose build
# run container
docker run -it --rm --name kaist_aim_container -v /root/TEAM_AIM:/root/TEAM_AIM kaist_aim

## 3. in docker ==================

## commands using sh
# mission_1_1
chmod +x /root/TEAM_AIM/mission_1_1.sh && ./mission_1_1.sh

# mission_1_2
chmod +x /root/TEAM_AIM/mission_1_2.sh && ./mission_1_2.sh

# mission_2
chmod +x /root/TEAM_AIM/mission_2.sh && ./mission_2.sh

# mission_3
chmod +x /root/TEAM_AIM/mission_3.sh && ./mission_3.sh



# 미션별 스크립트 설명

## mission_1_1
`mission_1_1.sh`는 mission_1_1의 실행을 위한 스크립트입니다. 실행 권한을 부여한 후 실행하면 mission_1_1 관련 노드가 실행됩니다.

## mission_1_2
`mission_1_2.sh`는 mission_1_2의 실행을 위한 스크립트입니다. 실행 권한을 부여한 후 실행하면 mission_1_2 관련 노드가 실행됩니다.

## mission_2
`mission_2.sh`는 mission_2의 모든 역할(시뮬레이터, CAV 1~4, 타워, 로터리)을 한 번에 병렬로 실행하는 자동화 스크립트입니다. 각 역할은 내부적으로 `entrypoint_ver2.sh`를 호출하여 환경변수에 따라 적절한 노드를 실행합니다. 실행 시 각 역할의 로그는 `log/mission_2_*.log`에 저장됩니다.

## mission_3
`mission_3.sh`는 mission_3의 모든 역할(시뮬레이터, CAV 1~4, 타워, 로터리)을 한 번에 병렬로 실행하는 자동화 스크립트입니다. 각 역할은 내부적으로 `entrypoint_ver2.sh`를 호출하여 환경변수에 따라 적절한 노드를 실행합니다. 실행 시 각 역할의 로그는 `log/mission_3_*.log`에 저장됩니다.