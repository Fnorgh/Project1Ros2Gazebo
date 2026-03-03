#!/usr/bin/env bash

# ROS2 networking defaults (LAB SAFE)
unset ROS_LOCALHOST_ONLY || true
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_SHM_TRANSPORT=0

# Source ROS + your workspace
source /opt/ros/jazzy/setup.bash
source "./install/setup.bash"
