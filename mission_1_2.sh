#!/bin/bash
set -e

cd ~/TEAM_AIM/
colcon build
source install/setup.bash

# Simulator
# ( PROBLEM_ID=2 ROLE=simulator ./entrypoint.sh ) &

# CAVs
( PROBLEM_ID=2 ROLE=cav CAV_ID=32 ./entrypoint.sh ) &
# ( PROBLEM_ID=2 ROLE=cav CAV_ID=2 ./entrypoint.sh ) &

# control tower
( PROBLEM_ID=2 ROLE=tower ./entrypoint.sh ) &

wait