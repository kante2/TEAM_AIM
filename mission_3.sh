#!/bin/bash
set -e

cd ~/TEAM_AIM/
colcon build
source install/setup.bash

# Launch all mission_3 nodes in separate terminals (gnome-terminal or xterm)
# Simulator
# ( PROBLEM_ID=4 ROLE=simulator ./entrypoint.sh ) &

# CAVs
( PROBLEM_ID=4 ROLE=cav CAV_ID=32 ./entrypoint.sh ) 
( PROBLEM_ID=4 ROLE=cav CAV_ID=1 ./entrypoint.sh ) &
( PROBLEM_ID=4 ROLE=cav CAV_ID=3 ./entrypoint.sh ) &
( PROBLEM_ID=4 ROLE=cav CAV_ID=6 ./entrypoint.sh ) &

# have to use cav_ids for using four cavs in tower and rotary !! KANTE
# Tower
( PROBLEM_ID=4 ROLE=tower CAV_IDS="32,1,3,6" ./entrypoint.sh ) &
# Rotary
( PROBLEM_ID=4 ROLE=rotary CAV_IDS="32,1,3,6" ./entrypoint.sh ) &

wait
