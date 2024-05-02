import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():

    publisher_aruco_node = launch_ros.actions.Node(
        package='aruco_pkg',
        executable='aruco.py',
        name='publisher',
        output='screen',
    )
    
    return launch.LaunchDescription([
        publisher_aruco_node
    ])