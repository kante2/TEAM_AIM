#!/bin/bash
set -e

cd ~/TEAM_AIM/
source install/setup.bash

# Launch all mission_3 nodes in separate terminals (gnome-terminal or xterm)
# Simulator
( PROBLEM_ID=1 ROLE=simulator ./entrypoint.sh ) &

# CAVs
( PROBLEM_ID=1 ROLE=cav CAV_ID=1 ./entrypoint.sh )

wait
