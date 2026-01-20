#!/bin/bash
# Mission 2 Domain Configuration

# Kill previous ROS processes
echo "[INFO] Killing previous ROS processes..."
killall -9 ros2 2>/dev/null || true
killall -9 control_cav_mission_1_1 2>/dev/null || true
killall -9 control_cav_mission_1_2 2>/dev/null || true
killall -9 control_cav_mission_2 2>/dev/null || true
killall -9 control_cav_mission_3 2>/dev/null || true
killall -9 simulator 2>/dev/null || true
killall -9 control_tower_mission_1_2 2>/dev/null || true
killall -9 control_tower_mission_3 2>/dev/null || true
killall -9 control_rotary_mission_3 2>/dev/null || true
sleep 1

# Set domain for Mission 2
export ROS_DOMAIN_ID=110
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "=== Mission 2 Domain Settings ==="
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "======================================="
