#!/usr/bin/env bash
# run_sim.sh - Build and launch TurtleBot4 simulation

set -eo pipefail  # (no -u here; we enable after sourcing)

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── 0. Source ROS env for bash (avoid nounset issues) ────────────────────────
set +u
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi

if [[ -f "$WORKSPACE/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$WORKSPACE/install/setup.bash"
fi
set -u

# ── 0b. ROS networking / DDS defaults (shared lab machines) ──────────────────
# Don't restrict discovery to localhost; it breaks multi-terminal discovery patterns.
unset ROS_LOCALHOST_ONLY || true
export ROS_DOMAIN_ID=0

# FastDDS shared-memory transport often fails on shared machines; disable it.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_SHM_TRANSPORT=0

# ── 1. Build ────────────────────────────────────────────────────────────────
echo "[run_sim] Building project_1..."
cd "$WORKSPACE"
colcon build --packages-select project_1
echo "[run_sim] Build complete."

# ── 2. Args ────────────────────────────────────────────────────────────────
X_POSE="${1:-0.0}"
Y_POSE="${2:-0.0}"
YAW="${3:-0.0}"

# ── 3. Launch simulation ────────────────────────────────────────────────────
echo "[run_sim] Launching simulation..."

gnome-terminal --title="Simulation" -- bash -lc "
set -eo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source \"$WORKSPACE/install/setup.bash\"
set -u

# Same env in the launched terminal
unset ROS_LOCALHOST_ONLY || true
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_SHM_TRANSPORT=0

ros2 launch project_1 simulation.launch.py x_pose:=$X_POSE y_pose:=$Y_POSE yaw:=$YAW
exec bash
"

echo "[run_sim] Done. Gazebo launching in a new terminal."