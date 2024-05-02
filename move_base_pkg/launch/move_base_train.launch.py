import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    coniguration_map = os.path.join(get_package_share_directory('move_base_pkg'),'config','map.yaml')


    publisher_node_move_base_train = launch_ros.actions.Node(
        package='move_base_pkg',
        executable='move_base_train_node',
        name='move_base_train_node',
        output='screen',
        parameters=[coniguration_map]

    )
    
    return launch.LaunchDescription([
        publisher_node_move_base_train
    ])

