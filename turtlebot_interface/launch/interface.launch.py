#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import IncludeLaunchDescription

from launch_ros.actions import Node

def generate_launch_description():
    
    viz = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('turtlebot_description'), 'launch'), '/visualize.launch.py']),)

    interface = Node(package='turtlebot_interface', executable='interface')
    
    return LaunchDescription([viz, interface])