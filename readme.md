#### Installation
colcon build --symlink-install
source install/setup.bash

#### Running
ros2 launch my_robot_arm robot_launch.py 
ros2 service call /set_joint_positions std_srvs/srv/SetBool "{data: true}"

