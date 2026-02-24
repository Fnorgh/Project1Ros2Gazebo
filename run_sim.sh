#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Always use the unified env
set +u
source "$WORKSPACE/env.sh"
set -u

echo "[run_sim] Building project_1..."
cd "$WORKSPACE"
colcon build --packages-select project_1
echo "[run_sim] Build complete."

X_POSE="${1:-0.0}"
Y_POSE="${2:-0.0}"
YAW="${3:-0.0}"

echo "[run_sim] Launching simulation..."

gnome-terminal --title="Simulation" -- bash -lc "
set -eo pipefail
set +u
source \"$WORKSPACE/env.sh\"
set -u
ros2 launch project_1 simulation.launch.py x_pose:=$X_POSE y_pose:=$Y_POSE yaw:=$YAW
exec bash
"

echo "[run_sim] Done. Gazebo launching in a new terminal."