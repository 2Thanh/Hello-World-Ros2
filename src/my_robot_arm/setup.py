from setuptools import setup

package_name = 'my_robot_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/arm.urdf.xacro']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thanhvl',
    maintainer_email='your_email@example.com',
    description='2-DOF Robot Arm Simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'joint_state_publisher = my_robot_arm.joint_state_publisher:main',
        'joint_command_publisher = my_robot_arm.joint_command_publisher:main',  # Added
    ],
},
)