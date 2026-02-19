import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/fnorgh/skeewl/project1Robot/Project1Ros2Gazebo/install/project_1'
