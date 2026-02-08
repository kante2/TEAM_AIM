#!/bin/bash
set -e

# domain
./mission_domain_4.sh

# ==========================================
# Build the workspace
# for REAL ROBOT (using /home/aim/TEAM_AIM/ path)
# ==========================================

cd /home/aim/TEAM_AIM/

colcon build
source install/setup.bash

# Launch all mission_3 nodes in separate terminals
# Set TEAM_AIM_HOME for real robot environment
export TEAM_AIM_HOME="/home/aim/TEAM_AIM"

# Four CAVs REAL ROBOT
# 32, 5, 9, 10
( PROBLEM_ID=4 ROLE=cav CAV_ID=32 CAV_IDS="32,5,9,10" TEAM_AIM_HOME="/home/aim/TEAM_AIM" ./entrypoint.sh ) & # path_mission3_01.csv
( PROBLEM_ID=4 ROLE=cav CAV_ID=5 CAV_IDS="32,5,9,10" TEAM_AIM_HOME="/home/aim/TEAM_AIM" ./entrypoint.sh ) & # path_mission3_02.csv
( PROBLEM_ID=4 ROLE=cav CAV_ID=9 CAV_IDS="32,5,9,10" TEAM_AIM_HOME="/home/aim/TEAM_AIM" ./entrypoint.sh ) & # path_mission3_03.csv
( PROBLEM_ID=4 ROLE=cav CAV_ID=10 CAV_IDS="32,5,9,10" TEAM_AIM_HOME="/home/aim/TEAM_AIM" ./entrypoint.sh ) & # path_mission3_04.csv

# Tower
( PROBLEM_ID=4 ROLE=tower CAV_IDS="32,5,9,10" TEAM_AIM_HOME="/home/aim/TEAM_AIM" ./entrypoint.sh ) &

# Rotary
( PROBLEM_ID=4 ROLE=rotary CAV_IDS="32,5,9,10" TEAM_AIM_HOME="/home/aim/TEAM_AIM" ./entrypoint.sh ) &

wait
