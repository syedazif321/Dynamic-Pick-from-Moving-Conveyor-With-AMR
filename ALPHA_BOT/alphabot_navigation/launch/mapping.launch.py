import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    pkg_bcr = get_package_share_directory('alphabot_navigation')
    pkg_rviz = get_package_share_directory('alphabot_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    autostart = LaunchConfiguration('autostart', default='True')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start the slam_toolbox stack'
    )
    slam_toolbox_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox_dir, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': os.path.join(pkg_bcr, 'config', 'mapper_params_online_async.yaml'),
        }.items()
    )

    rviz_launch_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            '-d', os.path.join(pkg_rviz, 'config', 'map.rviz')
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




    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(slam_toolbox_launch_cmd)
    ld.add_action(rviz_launch_cmd)
    ld.add_action(static_transform_publisher_node)
    ld.add_action(scan_filter_node)


    return ld

