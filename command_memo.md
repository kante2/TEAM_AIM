===========================================================================================================================
-

# host --> x server
xhost +local:docker

colcon build
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

# mission1

colcon build --packages-select mission_2
source install/setup.bash
export ROS_DOMAIN_ID=110
export CAV_ID=1
ros2 run mission_2 control_cav_mission_2

================================================================================
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
# mission_1_1 실행 방법

<CHECK VERSION MODE>
colcon build
export ROS_DOMAIN_ID=105
cd ~/TEAM_AIM/Mobility_Challenge_Simulator
source install/setup.bash
ros2 launch simulator_launch simulator_launch.py

# 4. 실행
export ROS_DOMAIN_ID=105
export CAV_ID=1
cd ~/TEAM_AIM
source install/setup.bash
ros2 run mission_1_1 control_cav_mission_1_1

================================================================================
================================================================================
# mission_1_2 실행 방법

<CHECK VERSION MODE>
colcon build
export ROS_DOMAIN_ID=107
cd ~/TEAM_AIM/Mobility_Challenge_Simulator
source install/setup.bash
ros2 launch simulator_launch simulator_launch.py

# CAV1
colcon build --packages-select mission_1_2
source install/setup.bash
export ROS_DOMAIN_ID=107
export CAV_ID=1
ros2 run mission_1_2 control_cav_mission_1_2

# CAV2
colcon build --packages-select mission_1_2
source install/setup.bash
export ROS_DOMAIN_ID=107
export CAV_ID=2
ros2 run mission_1_2 control_cav_mission_1_2

# CONTROL TOWER
colcon build --packages-select mission_1_2
source install/setup.bash
export ROS_DOMAIN_ID=107
ros2 run mission_1_2 control_tower_mission_1_2