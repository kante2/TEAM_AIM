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
# Four CAVs
( PROBLEM_ID=4 ROLE=cav CAV_ID=32 CAV_IDS="32,2,3,4" ./entrypoint.sh ) &
( PROBLEM_ID=4 ROLE=cav CAV_ID=2 CAV_IDS="32,2,3,4" ./entrypoint.sh ) &
( PROBLEM_ID=4 ROLE=cav CAV_ID=3 CAV_IDS="32,2,3,4" ./entrypoint.sh ) &
( PROBLEM_ID=4 ROLE=cav CAV_ID=4 CAV_IDS="32,2,3,4" ./entrypoint.sh ) &

# have to use cav_ids for using four cavs in tower and rotary !! KANTE
# Tower
( PROBLEM_ID=4 ROLE=tower CAV_IDS="32,2,3,4" ./entrypoint.sh ) &
# Rotary
( PROBLEM_ID=4 ROLE=rotary CAV_IDS="32,2,3,4" ./entrypoint.sh ) &

wait
