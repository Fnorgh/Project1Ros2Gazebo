#!/usr/bin/env bash
set -e

# Resolve workspace directory even if sourced from elsewhere
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---- 1) Source ROS + workspace FIRST ----
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
else
  echo "[env.sh] ERROR: /opt/ros/jazzy/setup.bash not found. Is ROS 2 Jazzy installed?" >&2
  return 1 2>/dev/null || exit 1
fi

# Prefer installed overlay if present, otherwise allow just /opt/ros
if [[ -f "$WORKSPACE/install/setup.bash" ]]; then
  source "$WORKSPACE/install/setup.bash"
fi

# ---- 2) Force lab-safe networking LAST so it always wins ----
unset ROS_LOCALHOST_ONLY || true
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# Force FastDDS RMW + disable SHM
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_SHM_TRANSPORT=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$WORKSPACE/fastdds_no_shm.xml"

# ---- 3) Resource paths for custom worlds/models ----
export GZ_SIM_RESOURCE_PATH="$WORKSPACE/worlds:${GZ_SIM_RESOURCE_PATH:-}"
export GAZEBO_MODEL_PATH="$WORKSPACE/models:${GAZEBO_MODEL_PATH:-}"

# ---- 4) gz_ros2_control plugin path ----
# Without this, Gazebo silently skips the plugin and controller_manager never starts.
# libgz_ros2_control-system.so lives in /opt/ros/jazzy/lib
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

# ---- 5) Quick sanity output ----
echo "[env.sh] GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH"