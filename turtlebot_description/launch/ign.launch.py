#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    pkg_turtlebot_desc = get_package_share_directory('turtlebot_description')

    ign = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'),),)

    spawn = Node(package='ros_ign_gazebo', executable='create', arguments=['-name', 'turtlebot', '-topic', '/robot_description'])

    ign_args = DeclareLaunchArgument('ign_args', default_value=[os.path.join(pkg_turtlebot_desc, 'worlds', 'turtlebot_world.sdf')])

    bridge = Node(
            package='ros_ign_bridge', 
            executable='parameter_bridge', 
            arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist','/model/turtlebot/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',])

    return LaunchDescription([ign_args, ign, spawn, bridge])