import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    uid = LaunchConfiguration('uid')

    imu_node = Node(
        package='tinkerforge_imu_ros',
        output='screen',
        executable='tinkerforge_imu_ros')

 
    return LaunchDescription([

        DeclareLaunchArgument('uid', default_value='6Det55',
                              description='Device uid'),
       
        imu_node
    ])
