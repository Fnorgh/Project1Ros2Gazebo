# Project1Ros2Gazebo


What's up chat

To run `gz sim Project1Ros2Gazebo/project_1/worlds/project_world.sdf`, please do the following while in `Project1Ros2Gazebo`:

```bash
export GAZEBO_MODEL_PATH="$PWD/project_1/models:${GAZEBO_MODEL_PATH}"
export GZ_SIM_RESOURCE_PATH="$PWD/project_1/models:${GZ_SIM_RESOURCE_PATH}"
```

## New stuff

```bash
pkill -f "gz sim"; or true
                           pkill -f "gzserver"; or true
                           pkill -f "gzclient"; or true
                           pkill -f "ign gazebo"; or true
                           pkill -f "ignition"; or true
                           pkill -f "ros_gz"; or true
```

for my code i use to run:

```bash
 bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch project_1 simulation.launch.py"
```
