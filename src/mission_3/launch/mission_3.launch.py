
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
	ld = LaunchDescription()

	# Control Tower
	tower_node = Node(
		package='mission_3',
		executable='control_tower_mission_3',
		name='control_tower',
		output='screen',
		parameters=[{'path_dir': '/root/TEAM_AIM/src/global_path/'}]
	)
	ld.add_action(tower_node)

	# Control Rotary (HV handling)
	rotary_node = Node(
		package='mission_3',
		executable='control_rotary_mission_3',
		name='control_rotary',
		output='screen',
		parameters=[{'path_dir': '/root/TEAM_AIM/src/global_path/'}]
	)
	ld.add_action(rotary_node)

	# Launch CAV control nodes (CAV_01 .. CAV_04)
	num_cavs = 4
	for i in range(1, num_cavs + 1):
		cav_id = i
		cav_name = f"control_cav_{cav_id:02d}"
		cav_node = Node(
			package='mission_3',
			executable='control_cav_mission_3',
			name=cav_name,
			output='screen',
			parameters=[{'cav_id': cav_id}]
		)
		ld.add_action(cav_node)

	return ld

