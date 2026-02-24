# Project1Ros2Gazebo


What's up chat

To run `gz sim Project1Ros2Gazebo/project_1/worlds/project_world.sdf`, please do the following while in `Project1Ros2Gazebo`:

```bash
export GAZEBO_MODEL_PATH="$PWD/project_1/models:${GAZEBO_MODEL_PATH}"
export GZ_SIM_RESOURCE_PATH="$PWD/project_1/models:${GZ_SIM_RESOURCE_PATH}"
```

## New stuff

```bass
pkill -f "gz sim"; or true
pkill -f "gzserver"; or true
pkill -f "gzclient"; or true
pkill -f "ign gazebo"; or true
pkill -f "ignition"; or true
pkill -f "ros_gz"; or true
```
Bash (best)
```bash
pkill -f "gz sim"      || true
pkill -f "gzserver"    || true
pkill -f "gzclient"    || true
pkill -f "ign gazebo"  || true
pkill -f "ignition"    || true
pkill -f "ros_gz"      || true
```


term 1
```bash
source ./env.sh
./run_sim.sh
```
term 2
```bash
source ./env.sh
ros2 run project_1 reactive_controller
```
term 3
```bash
source ./env.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_key
```

rebase and shit

```bash
colcon build --packages-select project_1
```
