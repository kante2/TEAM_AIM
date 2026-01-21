#!/bin/bash
set -e

# Usage:
#   PROBLEM_ID=2 ROLE=simulator ./entrypoint_ver2.sh
#   PROBLEM_ID=2 ROLE=cav CAV_ID=1 ./entrypoint_ver2.sh
#   PROBLEM_ID=2 ROLE=tower ./entrypoint_ver2.sh
#   PROBLEM_ID=3 ROLE=rotary ./entrypoint_ver2.sh

# Default values
PROBLEM_ID=${PROBLEM_ID:-2}
ROLE=${ROLE:-simulator}
CAV_ID=${CAV_ID:-1}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-110}

cd /root/TEAM_AIM
source /root/TEAM_AIM/install/setup.bash

# Set default network configuration for all missions
if [ -z "$ROS_LOCALHOST_ONLY" ]; then
    export ROS_LOCALHOST_ONLY=0
fi
if [ -z "$RMW_IMPLEMENTATION" ]; then
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
fi

# mission_2
if [ "$PROBLEM_ID" = "3" ]; then
    if [ "$ROLE" = "simulator" ]; then
        export ROS_DOMAIN_ID=110
        cd /root/TEAM_AIM/Mobility_Challenge_Simulator
        ros2 launch simulator_launch simulator_launch.py
    elif [ "$ROLE" = "cav" ]; then
        export ROS_DOMAIN_ID=110
        export CAV_ID=$CAV_ID
        ros2 run mission_2 control_cav_mission_2
    else
        echo "[ERROR] Unknown ROLE for mission_2: $ROLE" >&2
        exit 2
    fi
# mission_3
elif [ "$PROBLEM_ID" = "4" ]; then
    if [ "$ROLE" = "simulator" ]; then
        export ROS_DOMAIN_ID=100
        # export ROS_DOMAIN_ID=100
        cd /root/TEAM_AIM/Mobility_Challenge_Simulator
        ros2 launch simulator_launch simulator_launch.py
    elif [ "$ROLE" = "cav" ]; then
        # export ROS_DOMAIN_ID=101
        export ROS_DOMAIN_ID=100
        export CAV_ID=$CAV_ID
        ros2 run mission_3 control_cav_mission_3
    elif [ "$ROLE" = "tower" ]; then
        # export ROS_DOMAIN_ID=101
        export ROS_DOMAIN_ID=100
        export CAV_IDS=${CAV_IDS:-"1,2,3,4"}
        ros2 run mission_3 control_tower_mission_3
    elif [ "$ROLE" = "rotary" ]; then
        # export ROS_DOMAIN_ID=101
        export ROS_DOMAIN_ID=100
        export CAV_IDS=${CAV_IDS:-"1,2,3,4"}
        ros2 run mission_3 control_rotary_mission_3
    else
        echo "[ERROR] Unknown ROLE for mission_3: $ROLE" >&2
        exit 2
    fi
# mission_1_1
elif [ "$PROBLEM_ID" = "1" ]; then
    if [ "$ROLE" = "simulator" ]; then
        export ROS_DOMAIN_ID=106
        cd /root/TEAM_AIM/Mobility_Challenge_Simulator
        ros2 launch simulator_launch simulator_launch.py
    elif [ "$ROLE" = "cav" ]; then
        export ROS_DOMAIN_ID=106
        export CAV_ID=$CAV_ID
        ros2 run mission_1_1 control_cav_mission_1_1
    else
        echo "[ERROR] Unknown ROLE for mission_1_1 : $ROLE" >&2
        exit 2
    fi
# mission_1_2
elif [ "$PROBLEM_ID" = "2" ]; then
    if [ "$ROLE" = "simulator" ]; then
        export ROS_DOMAIN_ID=107
        cd /root/TEAM_AIM/Mobility_Challenge_Simulator
        ros2 launch simulator_launch simulator_launch.py
    elif [ "$ROLE" = "cav" ]; then
        export ROS_DOMAIN_ID=107
        export CAV_ID=$CAV_ID
        ros2 run mission_1_2 control_cav_mission_1_2
    elif [ "$ROLE" = "tower" ]; then
        export ROS_DOMAIN_ID=107
        ros2 run mission_1_2 control_tower_mission_1_2
    else
        echo "[ERROR] Unknown ROLE for mission_1_2 : $ROLE" >&2
        exit 2
    fi

else
    echo "[ERROR] Unknown PROBLEM_ID: $PROBLEM_ID" >&2
    exit 1
fi
