#!/usr/bin/env bash
unset ROS_LOCALHOST_ONLY || true
unset LIBGL_ALWAYS_SOFTWARE || true
unset MESA_LOADER_DRIVER_OVERRIDE || true

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_SHM_TRANSPORT=0
export QT_QPA_PLATFORM=xcb
export GZ_SIM_RENDER_ENGINE=ogre
export GZ_RENDERING_ENGINE=ogre

source /opt/ros/jazzy/setup.bash
source "$HOME/robotics/Project1Ros2Gazebo/install/setup.bash"

# ✅ world discovery by NAME (world:=enviromental)
export GZ_SIM_RESOURCE_PATH="$HOME/robotics/Project1Ros2Gazebo/install/project_1/share/project_1/worlds:${GZ_SIM_RESOURCE_PATH}"