#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    nav2_pkg = get_package_share_directory("nav2_bringup")
    alphabot_nav = get_package_share_directory("alphabot_navigation")

    params_file = os.path.join(alphabot_nav, "config", "nav2_params.yaml")
    map_file = os.path.join(alphabot_nav, "config", "floor_0.yaml") 

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_pkg, "launch", "bringup_launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "params_file": params_file,
            "map": map_file
        }.items()
    )

    return LaunchDescription([nav2])
