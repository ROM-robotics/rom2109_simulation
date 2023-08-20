#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    deepblue = Node(
        name="deepblue",
        package="rom_markers",
        executable="deepblue",
    )
    romrobotics = Node(
        name="romrobotics",
        package="rom_markers",
        executable="romrobotics",
    )
    
    return LaunchDescription(
        [
            deepblue,
            #romrobotics,
        ]
    )
