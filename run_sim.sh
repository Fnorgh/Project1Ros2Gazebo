#!/usr/bin/env bash
# run_sim.sh - Build and launch Gazebo + TurtleBot4 spawn (reliable order)

set -eo pipefail
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Source ROS + workspace ──────────────────────────────────────────────────
set +u
source /opt/ros/jazzy/setup.bash
if [[ -f "$WORKSPACE/install/setup.bash" ]]; then
  source "$WORKSPACE/install/setup.bash"
fi
set -u

# ── DDS / lab machine stability ─────────────────────────────────────────────
unset ROS_LOCALHOST_ONLY || true
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_SHM_TRANSPORT=0
export QT_QPA_PLATFORM=xcb
export GZ_SIM_RENDER_ENGINE=ogre
export GZ_RENDERING_ENGINE=ogre

# ── Build ───────────────────────────────────────────────────────────────────
echo "[run_sim] Building project_1..."
cd "$WORKSPACE"
colcon build --packages-select project_1
echo "[run_sim] Build complete."

WORLD_FILE="$WORKSPACE/install/project_1/share/project_1/worlds/enviromental.sdf"
WORLD_NAME="project_world"   # <-- from <world name='project_world'>

echo "[run_sim] Launching Gazebo world: $WORLD_FILE"

gnome-terminal --title="Gazebo" -- bash -lc "
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source \"$WORKSPACE/install/setup.bash\"

unset ROS_LOCALHOST_ONLY || true
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_SHM_TRANSPORT=0
export QT_QPA_PLATFORM=xcb
export GZ_SIM_RENDER_ENGINE=ogre
export GZ_RENDERING_ENGINE=ogre

ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=\"-r -v 4 $WORLD_FILE\"
exec bash
"

echo "[run_sim] Waiting for Gazebo to come up..."
sleep 4

echo "[run_sim] Spawning TurtleBot4 into world: $WORLD_NAME"
gnome-terminal --title="TB4 Spawn" -- bash -lc "
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source \"$WORKSPACE/install/setup.bash\"

unset ROS_LOCALHOST_ONLY || true
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_SHM_TRANSPORT=0

ros2 launch turtlebot4_gz_bringup turtlebot4_spawn.launch.py gazebo:=ignition world:=$WORLD_NAME
exec bash
"

echo "[run_sim] Done."