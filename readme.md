#### Installation
colcon build --symlink-install
source install/setup.bash

#### Running
ros2 launch my_robot_arm robot_launch.py 

#### Send the specific angle of each joint
python3 send.py

