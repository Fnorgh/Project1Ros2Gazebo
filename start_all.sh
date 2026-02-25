#!/usr/bin/env bash
set -eo pipefail

# Resolve this script's directory (same folder as env.sh)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Source env (sets WORKSPACE, ROS, networking, GZ paths) ────────────────
source "$SCRIPT_DIR/env.sh"
# WORKSPACE is now set by env.sh and equals SCRIPT_DIR

# ── Build project_1 ────────────────────────────────────────────────────────
echo "[start_all] Building project_1..."
cd "$WORKSPACE"
colcon build --packages-select project_1 --symlink-install
# Re-source so the freshly built install overlay is active
source "$WORKSPACE/install/setup.bash"
echo "[start_all] Build complete."

# ── Start simulation ────────────────────────────────────────────────────────
"$WORKSPACE/run_sim.sh" "$@"

# ── Teleop keyboard ─────────────────────────────────────────────────────────
gnome-terminal --title="Teleop" -- bash -lc "
source \"$WORKSPACE/env.sh\"
echo '[Teleop] Use WASD / arrow keys to drive. Ctrl+C to stop.'
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/cmd_vel_key -p use_sim_time:=true
exec bash
"

echo "[start_all] All components launched."