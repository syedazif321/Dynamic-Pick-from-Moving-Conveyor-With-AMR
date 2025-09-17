#!/usr/bin/env python3
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, FindExecutable, Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    # --- Package paths ---
    pkg_desc = get_package_share_directory("mobile_manipulator_description")
    pkg_gazebo = get_package_share_directory("alphabot_gazebo")
    pkg_moveit = get_package_share_directory("rm_75_config")
    pkg_arm = get_package_share_directory("rm_description")
 
    srdf_file = os.path.join(pkg_moveit, "config", "rm_75_description.srdf")
    kinematics_yaml_file = os.path.join(pkg_moveit, "config", "kinematics.yaml")
    full_urdf_file = os.path.join(pkg_desc, "urdf", "mobile_manipulator.urdf.xacro")
    arm_urdf_file = os.path.join(pkg_arm, "urdf", "rm_75_gazebo.urdf")
    
    

    # --- Load YAML ---
    with open(kinematics_yaml_file, 'r') as f:
        kinematics_dict = yaml.safe_load(f)

    # --- Launch arguments ---
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([pkg_gazebo, "worlds", "no_roof_small_warehouse.world"]),
        description="SDF world file"
    )
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    entity_arg = DeclareLaunchArgument("entity", default_value="mobile_manipulator")

    # --- Robot description for Gazebo ---
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", full_urdf_file]), value_type=None
        )
    }

    # --- Robot description for pick_controller (arm only) ---
    arm_robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", arm_urdf_file]), value_type=None
        )
    }

    # --- Gazebo launch ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, "launch", "gazebo.launch.py")),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )

    # --- Robot state publisher (full robot) ---
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen"
    )

    # --- Spawn robot in Gazebo ---
    spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", LaunchConfiguration("entity"), "-topic", "robot_description"],
        output="screen"
    )


    # --- Controllers ---
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen"
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rm_group_controller", "-c", "/controller_manager"],
        output="screen"
    )
    slider_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["slider_position_controller", "-c", "/controller_manager"],
        output="screen"
    )

    # --- Event chaining for controllers ---
    load_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawner, on_exit=[jsb_spawner])
    )
    load_arm = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner])
    )
    load_slider = RegisterEventHandler(
        OnProcessExit(target_action=arm_spawner, on_exit=[slider_spawner])
    )

    # --- Static TFs ---
    # static_tf_camera_mount = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='tf_camera_mount',
    #     arguments=['-0.675734', '17.9172', '1.75021', '0', '0', '-1.957', 'world', 'realsense_rgb_frame']
    # )
    # static_tf_map_to_odom = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='map_to_odom',
    #     output='screen',
    #     arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom']
    # )
    # static_tf_world_to_map = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='world_to_map',
    #     output='screen',
    #     arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    # )

    # --- Vision node ---
    vision_node = Node(
        package='pipeline_manipulator',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    # --- Pick controller node (arm-only URDF + SRDF + kinematics) ---
    pick_controller_node = Node(
        package='pipeline_manipulator',
        executable='pick_controller',
        name='pick_controller',
        output='screen',
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            arm_robot_description,
            {"robot_description_semantic": open(srdf_file).read()},
            kinematics_dict
        ]
    )

    return LaunchDescription([
        world_arg, use_sim_time_arg, entity_arg,
        gazebo,
        rsp,
        spawner,
        load_jsb,
        load_arm,
        load_slider,
        # static_tf_camera_mount,
        # static_tf_map_to_odom,
        # static_tf_world_to_map,
        vision_node,
        pick_controller_node
    ])
