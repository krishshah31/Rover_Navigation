from setuptools import find_packages, setup

package_name = 'motor_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='krish',
    maintainer_email='krish@todo.todo',
    description='Motor control package for controlling a rover using RoboClaw',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_control_node = motor_control.motor_control_node:main',
            'motor_command_publisher = motor_control.motor_command_publisher:main',
            'keyboard_control = motor_control.keyboard_control:main',
            'RoboClawCmdVelNode = motor_control.RoboClawCmdVelNode:main',
            'odometry_node = motor_control.odometry_node:main',
        ],
    },
)
