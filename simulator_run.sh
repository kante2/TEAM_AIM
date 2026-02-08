#!/bin/bash
set -e

colcon build
export ROS_DOMAIN_ID=102
cd /home/aim/TEAM_AIM/Mobility_Challenge_Simulator
source /home/aim/TEAM_AIM/install/setup.bash
ros2 launch simulator_launch simulator_launch.py
