# Project1Ros2Gazebo

## Run project

```bass
pkill -f "gz sim"; or true
pkill -f "gzserver"; or true
pkill -f "gzclient"; or true
pkill -f "ign gazebo"; or true
pkill -f "ignition"; or true
pkill -f "ros_gz"; or true
```
```bash
pkill -f "gz sim"      || true
pkill -f "gzserver"    || true
pkill -f "gzclient"    || true
pkill -f "ign gazebo"  || true
pkill -f "ignition"    || true
pkill -f "ros_gz"      || true
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
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_key
```
Rebase
```bash
colcon build --packages-select project_1
```

## Run world only
To run `gz sim Project1Ros2Gazebo/project_1/worlds/project_world.sdf`, please do the following while in `Project1Ros2Gazebo`:

```bash
export GAZEBO_MODEL_PATH="$PWD/project_1/models:${GAZEBO_MODEL_PATH}"
export GZ_SIM_RESOURCE_PATH="$PWD/project_1/models:${GZ_SIM_RESOURCE_PATH}"
```

# Plan
## Jace
* #2: Accept keyboard movement commands from a human user.
  * Fix teleop in terminal 3.
* #6: Drive forward.
* Map the world (not related to any behavior. Just create map)

## Noah
* #3: Escape from (roughly) symmetric obstacles within 1ft in front of the robot.
* #4: Avoid asymmetric obstacles within 1ft in front of the robot.
* #5: Turn randomly (uniformly sampled within ±15°) after every 1ft of forward movement.

## Both
* World/setup documentation
* Code documentation
  * Each write the documenation for their own or familiar code

