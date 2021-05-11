#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():

    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')

    pkg_turtlebot = get_package_share_directory('turtlebot_description')

    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'),),)

    model_path = os.path.join(pkg_turtlebot, 'models', 'turtlebot3_burger', 'model.sdf')

    spawn = Node(package='ros_ign_gazebo', executable='create', arguments=['-name', 'turtlebot', '-file', model_path])

    ign_args = DeclareLaunchArgument('ign_args', default_value=[os.path.join(pkg_turtlebot, 'worlds', 'turtlebot.sdf')])
    
    bridge = Node(
            package='ros_ign_bridge', 
            executable='parameter_bridge', 
            arguments=['/cmd_vel_robot@geometry_msgs/msg/Twist@ignition.msgs.Twist', '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry'])

    return LaunchDescription([ign_args, gazebo, spawn, bridge])

