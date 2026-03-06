# Project1Ros2Gazebo

## Run project

```bass
pkill -f "reactive_controller"; or true
pkill -f "gz sim"; or true
pkill -f "gzserver"; or true
pkill -f "gzclient"; or true
pkill -f "ign gazebo"; or true
pkill -f "ignition"; or true
pkill -f "ros_gz"; or true
```
```bash
pkill -f "reactive_controller" || true
pkill -f "gz sim"              || true
pkill -f "gzserver"            || true
pkill -f "gzclient"            || true
pkill -f "ign gazebo"          || true
pkill -f "ignition"            || true
pkill -f "ros_gz"              || true
```


Terminal 1
```bash
source ./env.sh
./run_sim.sh
```
Terminal 2
```bash
source ./env.sh
ros2 run project_1 reactive_controller --ros-args -p cmd_vel_topic:=/cmd_vel
```
Terminal 3
```bash
source ./env.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r /cmd_vel:=/cmd_vel
```

## Run world only
To run `gz sim Project1Ros2Gazebo/project_1/worlds/project_world.sdf`, please do the following while in `Project1Ros2Gazebo`:

```bash
export GAZEBO_MODEL_PATH="$PWD/project_1/models:${GAZEBO_MODEL_PATH}"
export GZ_SIM_RESOURCE_PATH="$PWD/project_1/models:${GZ_SIM_RESOURCE_PATH}"
```
