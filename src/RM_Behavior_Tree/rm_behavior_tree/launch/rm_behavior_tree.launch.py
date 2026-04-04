import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    bt_config_dir = os.path.join(get_package_share_directory('rm_behavior_tree'), 'config')
    
    # Default to an existing tree file in the package config directory
    style = LaunchConfiguration('style', default='3v3_new.xml')
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    bt_xml_file = PathJoinSubstitution([bt_config_dir, style])

    rm_behavior_tree_node = Node(
        package='rm_behavior_tree',
        executable='rm_behavior_tree',
        respawn=True,
        respawn_delay=3,
        parameters=[
                        {
                            'style': bt_xml_file,
                            'use_sim_time': use_sim_time,
                        }
        ]
    )

    return LaunchDescription([rm_behavior_tree_node])
