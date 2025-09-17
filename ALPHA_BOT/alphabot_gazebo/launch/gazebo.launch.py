#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    gazebo_pkg = get_package_share_directory("gazebo_ros")
    alphabot_gazebo = get_package_share_directory("alphabot_gazebo")

    world_file = LaunchConfiguration(
        "world",
        default=os.path.join(alphabot_gazebo, "worlds", "no_roof_small_warehouse.world")
    )

    return LaunchDescription([
        AppendEnvironmentVariable(
            name="GAZEBO_MODEL_PATH",
            value=os.path.join(alphabot_gazebo, "models")),
        SetEnvironmentVariable(
            name="GAZEBO_RESOURCE_PATH",
            value="/usr/share/gazebo-11:" + os.path.join(alphabot_gazebo, "worlds")),
        DeclareLaunchArgument("world", default_value=world_file),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_pkg, "launch", "gazebo.launch.py")),
            launch_arguments={"world": world_file}.items()
        )
    ])
