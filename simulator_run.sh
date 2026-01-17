#!/bin/bash
set -e

colcon build
export ROS_DOMAIN_ID=101
cd ~/TEAM_AIM/Mobility_Challenge_Simulator
source ~/TEAM_AIM/install/setup.bash
ros2 launch simulator_launch simulator_launch.py
