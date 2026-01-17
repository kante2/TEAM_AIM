
# COMMANDS  TEAM :AIM ======================================================
# mission_1_1
chmod +x /root/TEAM_AIM/mission_1_1.sh && ./mission_1_1.sh

# mission_1_2
chmod +x /root/TEAM_AIM/mission_1_2.sh && ./mission_1_2.sh

# mission_2
chmod +x /root/TEAM_AIM/mission_2.sh && ./mission_2.sh

# mission_3
chmod +x /root/TEAM_AIM/mission_3.sh && ./mission_3.sh


## mission_2 자동 실행 스크립트
`mission_2.sh`는 mission_2의 모든 역할(시뮬레이터, CAV 1~4, 타워, 로터리)을 한 번에 병렬로 실행하는 쉘 스크립트입니다.
각 역할은 내부적으로 `entrypoint_ver2.sh`를 호출하여 환경변수(PROBLEM_ID, ROLE, CAV_ID)에 따라 적절한 노드를 실행합니다.

### 실행 방법
1. 실행 권한 부여: `chmod +x /root/TEAM_AIM/mission_2.sh`
2. 실행: `./mission_2.sh`
3. 각 역할의 로그는 `log/mission_2_*.log` 파일에 저장됩니다.

---
# mission_3
chmod +x /root/TEAM_AIM/mission_3.sh && ./mission_3.sh

## mission_3 자동 실행 스크립트
`mission_3.sh`는 mission_3의 모든 역할(시뮬레이터, CAV 1~4, 타워, 로터리)을 한 번에 병렬로 실행하는 쉘 스크립트입니다.
각 역할은 내부적으로 `entrypoint_ver2.sh`를 호출하여 환경변수(PROBLEM_ID, ROLE, CAV_ID)에 따라 적절한 노드를 실행합니다.

### 실행 방법
1. 실행 권한 부여: `chmod +x /root/TEAM_AIM/mission_3.sh`
2. 실행: `./mission_3.sh`
3. 각 역할의 로그는 `log/mission_3_*.log` 파일에 저장됩니다.