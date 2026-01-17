#!/bin/bash
set -e

# Launch all mission_3 nodes in separate terminals (gnome-terminal or xterm)
# Simulator
( PROBLEM_ID=3 ROLE=simulator ./entrypoint.sh ) &

# CAVs
( PROBLEM_ID=3 ROLE=cav CAV_ID=1 ./entrypoint.sh ) &
( PROBLEM_ID=3 ROLE=cav CAV_ID=2 ./entrypoint.sh ) &
( PROBLEM_ID=3 ROLE=cav CAV_ID=3 ./entrypoint.sh ) &
( PROBLEM_ID=3 ROLE=cav CAV_ID=4 ./entrypoint.sh ) &

# Tower
( PROBLEM_ID=3 ROLE=tower ./entrypoint.sh ) &

# Rotary
( PROBLEM_ID=3 ROLE=rotary ./entrypoint.sh ) &

wait
