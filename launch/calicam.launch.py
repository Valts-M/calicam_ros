from ament_index_python.packages import get_package_share_directory

from launch.launch_description import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import os

def generate_launch_description():

    share_dir = get_package_share_directory('calicam_ros')
    parameter_file = LaunchConfiguration('params_file')
    node_name = LaunchConfiguration('node_name')
    
    params_declare = DeclareLaunchArgument('params_file',
                                        default_value=os.path.join(
                                            share_dir, 'params', 'calicam.yaml'),
                                            description='Full path to the paramater file')
    node_name_declare = DeclareLaunchArgument('node_name', default_value='calicam')
    
    calicam = Node(
        package='calicam_ros',
        executable='calicam',
        name=node_name,
        parameters=[parameter_file],
    )
    
    ld = LaunchDescription()

    ld.add_action(params_declare)
    ld.add_action(node_name_declare)
    ld.add_action(calicam)

    return ld
    
    
