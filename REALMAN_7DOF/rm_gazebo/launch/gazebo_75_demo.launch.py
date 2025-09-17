import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    package_name = 'rm_gazebo'
    robot_name_in_model = 'rm_75_description'

    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, 'config', 'gazebo_75_description.urdf.xacro')

    # Parse xacro → robot_description
    doc = xacro.parse(open(urdf_model_path))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    # 1) Start Gazebo Classic
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # 2) Robot State Publisher (publishes /robot_description)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True}, params, {"publish_frequency": 15.0}],
        output='screen'
    )

    # 3) Spawn entity from /robot_description
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robot_name_in_model],
        output='screen'
    )

    # 4) Controller spawners (they wait for /controller_manager to be up)
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_jsb',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    traj_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_rm_group',
        arguments=['rm_group_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Sequence: after spawn_entity finishes → start JSB spawner
    after_spawn_start_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[jsb_spawner]
        )
    )

    # After JSB spawns → start trajectory controller spawner
    after_jsb_start_traj = RegisterEventHandler(
        OnProcessExit(
            target_action=jsb_spawner,
            on_exit=[traj_spawner]
        )
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        after_spawn_start_jsb,
        after_jsb_start_traj,
    ])
