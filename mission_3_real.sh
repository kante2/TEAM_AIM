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
# 1,2,3,4
( PROBLEM_ID=4 ROLE=cav CAV_ID=1 CAV_IDS="1,2,3,4" TEAM_AIM_HOME="/home/aim/TEAM_AIM" ./entrypoint.sh ) & # path_mission3_01.csv
( PROBLEM_ID=4 ROLE=cav CAV_ID=2 CAV_IDS="1,2,3,4" TEAM_AIM_HOME="/home/aim/TEAM_AIM" ./entrypoint.sh ) & # path_mission3_02.csv
( PROBLEM_ID=4 ROLE=cav CAV_ID=3 CAV_IDS="1,2,3,4" TEAM_AIM_HOME="/home/aim/TEAM_AIM" ./entrypoint.sh ) & # path_mission3_03.csv
( PROBLEM_ID=4 ROLE=cav CAV_ID=4 CAV_IDS="1,2,3,4" TEAM_AIM_HOME="/home/aim/TEAM_AIM" ./entrypoint.sh ) & # path_mission3_04.csv

# Tower
( PROBLEM_ID=4 ROLE=tower CAV_IDS="1,2,3,4" TEAM_AIM_HOME="/home/aim/TEAM_AIM" ./entrypoint.sh ) &

# Rotary
( PROBLEM_ID=4 ROLE=rotary CAV_IDS="1,2,3,4" TEAM_AIM_HOME="/home/aim/TEAM_AIM" ./entrypoint.sh ) &

wait
