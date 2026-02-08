#!/bin/bash
set -e

# Motor check script - specify CAV_ID as argument
# Usage: ./debug_motor.sh <CAV_ID>
# Example: ./debug_motor.sh 1

CAV_ID=${1:-1}  # Default to 1 if not specified

echo "Building TEAM_AIM workspace..."
cd /home/aim/TEAM_AIM
colcon build --symlink-install --packages-select mission_3

echo "Sourcing setup..."
source /home/aim/TEAM_AIM/install/setup.bash

echo "======================================"
echo "Running check_motor with CAV_ID=$CAV_ID"
echo "======================================"
export CAV_ID=$CAV_ID
ros2 run mission_3 check_motor


# CAV1
#./debug_motor.sh 1

# CAV2
# ./debug_motor.sh 2

# CAV3
#./debug_motor.sh 3

# CAV4
# ./debug_motor.sh 4