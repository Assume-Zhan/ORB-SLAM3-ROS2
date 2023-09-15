import os

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, FindExecutable


def generate_launch_description():

    octomap = Node(
        parameters=[
          {
              'use_sim_time': True,
              'resolution': 0.01,
              'point_cloud_max_z': 0.3,
              'point_cloud_min_z': 0.0
          }
        ],
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server'
    )
    # -------------------------------
    
    # Launch description
    return LaunchDescription([
        octomap
    ])
    # -------------------------------