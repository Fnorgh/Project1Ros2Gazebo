#!/usr/bin/env bash
unset ROS_LOCALHOST_ONLY || true
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_SHM_TRANSPORT=0
source /opt/ros/jazzy/setup.bash
source "$HOME/robotics/Project1Ros2Gazebo/install/setup.bash"
