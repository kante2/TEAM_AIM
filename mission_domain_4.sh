#!/bin/bash
# Mission 3 Domain Configuration


# Set domain for Mission 3
export ROS_DOMAIN_ID=100 # 100 -> 101
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "=== Mission 3 Domain Settings ==="
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"
echo "RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
echo "======================================="
