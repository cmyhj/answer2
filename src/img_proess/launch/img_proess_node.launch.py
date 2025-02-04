import os

import launch
import launch.actions
import launch.events

import launch_ros
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    ld = launch.LaunchDescription()

    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('robot_bring_up'),
            'config',
            'nac.yaml')
    )
    declare_yaml_path = DeclareLaunchArgument(
        'param_dir',
        default_value=param_dir,
        description='Full path to the configuration file to load'
    )
    imu_data_transformer = Node(
        name='imu_data_transformer',
        namespace="",
        package='imu_transformer',
        executable='imu_transformer_node',
        output='screen',
        parameters=[param_dir]
    )

    ld.add_action(declare_yaml_path)
    ld.add_action(imu_data_transformer)

    return ld