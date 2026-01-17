===========================================================================================================================
-

# host --> x server
xhost +local:docker

colcon build
================================================================================
# in docker
# ---mission-3
# Terminal 0: Simulator

<CHECK VERSION MODE>
colcon build
export ROS_DOMAIN_ID=101
cd ~/TEAM_AIM/Mobility_Challenge_Simulator
source install/setup.bash
ros2 launch simulator_launch simulator_launch.py

# Terminal 1: CAV_01
cd ~/TEAM_AIM
export ROS_DOMAIN_ID=101
export CAV_ID=1
source install/setup.bash
source /root/TEAM_AIM/install/setup.bash && ros2 run mission_3 control_cav_mission_3


	source /root/TEAM_AIM/install/setup.bash && ros2 run mission_3 control_cav_mission_3
# Terminal 2: CAV_02
cd ~/TEAM_AIM
export ROS_DOMAIN_ID=101
export CAV_ID=2
source /root/TEAM_AIM/install/setup.bash && ros2 run mission_3 control_cav_mission_3

	source /root/TEAM_AIM/install/setup.bash && ros2 run mission_3 control_cav_mission_3

# Terminal 3: CAV_03
cd ~/TEAM_AIM
export ROS_DOMAIN_ID=101
export CAV_ID=3
source /root/TEAM_AIM/install/setup.bash && ros2 run mission_3 control_cav_mission_3

# Terminal 4: CAV_04
cd ~/TEAM_AIM
export ROS_DOMAIN_ID=101
export CAV_ID=4
source /root/TEAM_AIM/install/setup.bash
ros2 run mission_3 control_cav_mission_3


# Terminal : control tower
cd ~/TEAM_AIM
source /root/TEAM_AIM/install/setup.bash
export ROS_DOMAIN_ID=101
	source /root/TEAM_AIM/install/setup.bash && ros2 run mission_3 control_tower_mission_3

# Terminal : control rotary tower
cd ~/TEAM_AIM
source /root/TEAM_AIM/install/setup.bash
export ROS_DOMAIN_ID=101
	source /root/TEAM_AIM/install/setup.bash && ros2 run mission_3 control_rotary_mission_3
    
================================================================================
================================================================================
# ---mission-2
# Terminal 0: Simulatora

<CHECK VERSION MODE>
colcon build
export ROS_DOMAIN_ID=110
cd ~/TEAM_AIM/Mobility_Challenge_Simulator
source install/setup.bash
ros2 launch simulator_launch simulator_launch.py

# mission2 (Refactored - Simplified variable-based ROI detection)

colcon build --packages-select mission_2
source install/setup.bash
export ROS_DOMAIN_ID=110
export CAV_ID=1
ros2 run mission_2 control_cav_mission_2
