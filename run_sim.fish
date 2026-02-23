#!/usr/bin/env fish
# run_sim.fish - Build and launch TurtleBot4 simulation (manual control via Gazebo side panel)
#
# Usage:
#   fish run_sim.fish [x_pose] [y_pose] [yaw]
#
# Examples:
#   fish run_sim.fish                  # spawn at origin
#   fish run_sim.fish 1.0 2.0 1.57    # custom spawn pose

set WORKSPACE (cd (dirname (status filename)); and pwd)

# ── 0. Source ROS env for fish shell ─────────────────────────────────────────
if test -f /opt/ros/jazzy/setup.fish
    source /opt/ros/jazzy/setup.fish
end

if test -f $WORKSPACE/install/setup.fish
    source $WORKSPACE/install/setup.fish
end

# ── 1. Build ──────────────────────────────────────────────────────────────────
echo "[run_sim] Building project_1..."
cd $WORKSPACE
colcon build --packages-select project_1
or begin
    echo "[run_sim] Build FAILED. Aborting."
    exit 1
end
echo "[run_sim] Build complete."

# ── 2. Args ───────────────────────────────────────────────────────────────────
set X_POSE (test (count $argv) -ge 1; and echo $argv[1]; or echo "0.0")
set Y_POSE (test (count $argv) -ge 2; and echo $argv[2]; or echo "0.0")
set YAW    (test (count $argv) -ge 3; and echo $argv[3]; or echo "0.0")

set SOURCE "export ROS_LOCALHOST_ONLY=1 && export ROS_DOMAIN_ID=0 && source /opt/ros/jazzy/setup.bash && source $WORKSPACE/install/setup.bash"

# ── 3. Launch simulation ──────────────────────────────────────────────────────
echo "[run_sim] Launching simulation..."

gnome-terminal --title="Simulation" -- bash -c "
    $SOURCE
    ros2 launch project_1 simulation.launch.py \
        x_pose:=$X_POSE y_pose:=$Y_POSE yaw:=$YAW
    exec bash"

echo "[run_sim] Done. Gazebo launching in a new terminal (manual control mode)."
echo "[run_sim] Use the TurtleBot4 side panel in Gazebo (warehouse-style controls)."
echo "[run_sim] If robot is docked, click Undock in that panel before driving."
echo "[run_sim] Fish tip: use 'source install/setup.fish' (not install/setup.bash)."
