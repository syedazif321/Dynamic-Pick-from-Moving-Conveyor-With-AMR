#!/usr/bin/env python3

from os.path import join


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch.actions import AppendEnvironmentVariable

def generate_launch_description():

    mobile_manipulator_path = get_package_share_directory('mobile_manipulator_bringup')
    world_path = get_package_share_directory('alphabot_gazebo')
    nav_path = get_package_share_directory('alphabot_navigation')

    # Retrieve launch configuration arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    
    world_file = LaunchConfiguration("world_file", default = join(world_path, 'worlds', 'no_roof_small_warehouse.world'))
    
    # Include the Gazebo launch file
    gazebo_share = get_package_share_directory("gazebo_ros")
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(join(gazebo_share, "launch", "gazebo.launch.py"))
    # )

    spawn_mobile_manipulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(mobile_manipulator_path, "launch", "bringup_mobile_manipulator.launch.py")),
    )

    nav2_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(join(nav_path, "launch", 'nav2.launch.py')),
    )

    return LaunchDescription([
        # Declare launch arguments
        
        AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=join(mobile_manipulator_path, "models")),

        SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value="/usr/share/gazebo-11:" + join(mobile_manipulator_path, "worlds")),
        DeclareLaunchArgument('world', default_value = world_file),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value = use_sim_time),
        spawn_mobile_manipulator,
        nav2_launch
    ])
