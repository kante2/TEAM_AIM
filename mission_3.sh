#!/bin/bash
set -e

# domain
./mission_domain_4.sh

# Build the workspace
cd ~/TEAM_AIM/
colcon build
source install/setup.bash

# Launch all mission_3 nodes in separate terminals (gnome-terminal or xterm)
# Simulator
# ( PROBLEM_ID=4 ROLE=simulator ./entrypoint.sh ) &

# CAVs

# Four CAVs SIMULATOR
# 1,2,3,4
# ( PROBLEM_ID=4 ROLE=cav CAV_ID=1 CAV_IDS="1,2,3,4" ./entrypoint.sh ) & # path_mission3_01.csv
# ( PROBLEM_ID=4 ROLE=cav CAV_ID=2 CAV_IDS="1,2,3,4" ./entrypoint.sh ) & # path_mission3_02.csv
# ( PROBLEM_ID=4 ROLE=cav CAV_ID=3 CAV_IDS="1,2,3,4" ./entrypoint.sh ) & # path_mission3_03.csv
# ( PROBLEM_ID=4 ROLE=cav CAV_ID=4 CAV_IDS="1,2,3,4" ./entrypoint.sh ) & # path_mission3_04.csv

# # have to use cav_ids for using four cavs in tower and rotary !! KANTE
# # Tower
# ( PROBLEM_ID=4 ROLE=tower CAV_IDS="1,2,3,4" ./entrypoint.sh ) &
# # Rotary
# ( PROBLEM_ID=4 ROLE=rotary CAV_IDS="1,2,3,4" ./entrypoint.sh ) &


# # # Four CAVs REAL ROBOT
# # # 32, 5, 9, 10
( PROBLEM_ID=4 ROLE=cav CAV_ID=32 CAV_IDS="32, 5, 9, 10" ./entrypoint.sh ) & # path_mission3_01.csv
( PROBLEM_ID=4 ROLE=cav CAV_ID=5 CAV_IDS="32, 5, 9, 10" ./entrypoint.sh ) & # path_mission3_02.csv
( PROBLEM_ID=4 ROLE=cav CAV_ID=9 CAV_IDS="32, 5, 9, 10" ./entrypoint.sh ) & # path_mission3_03.csv
( PROBLEM_ID=4 ROLE=cav CAV_ID=10 CAV_IDS="32, 5, 9, 10" ./entrypoint.sh ) & # path_mission3_04.csv

# have to use cav_ids for using four cavs in tower and rotary !! KANTE
# Tower
( PROBLEM_ID=4 ROLE=tower CAV_IDS="32,5,9,10" ./entrypoint.sh ) &
# Rotary
( PROBLEM_ID=4 ROLE=rotary CAV_IDS="32,5,9,10" ./entrypoint.sh ) &

wait
