from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    pkg_bcr = get_package_share_directory('alphabot_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    autostart = LaunchConfiguration('autostart', default='True')

    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'map': os.path.join(pkg_bcr, 'config', 'floor_0.yaml'),
            'params_file': os.path.join(pkg_bcr, 'config', 'nav2_params.yaml'),
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            os.path.join(pkg_nav2_dir, 'rviz', 'nav2_default_view.rviz')
        ]
    )

    static_transform_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    scan_filter_node = Node(
        package='alphabot_navigation',   
        executable='scan_filter_node.py', 
        name='scan_filter_node',            
        output='screen',                    
        emulate_tty=True   
    )



    map_switcher_node = Node(
        package='alphabot_navigation',
        executable='map_switcher',
        name='map_switcher',
        output='screen'
    )


    ld = LaunchDescription()
    ld.add_action(nav2_launch_cmd)
    ld.add_action(rviz_node)
    ld.add_action(static_transform_publisher_node)
    ld.add_action(scan_filter_node)
    ld.add_action(map_switcher_node)

    return ld
