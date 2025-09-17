#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder
from srdfdom.srdf import SRDF
import os


def generate_launch_description():
    # ----------- Launch Args -----------
    use_rviz = DeclareLaunchArgument("use_rviz", default_value="true")
    use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    pause_gazebo = DeclareLaunchArgument("pause_gazebo", default_value="false")
    world = DeclareLaunchArgument(
        "world", default_value="",
        description="Path to Gazebo world. Leave empty for default empty.world"
    )
    rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([FindPackageShare("rm_75_config"), "config", "moveit.rviz"])
    )
    publish_frequency = DeclareLaunchArgument("publish_frequency", default_value="15.0")
    entity_name = DeclareLaunchArgument("entity_name", default_value="rm_75")
    db = DeclareLaunchArgument("db", default_value="false")

    # ----------- MoveIt Config -----------
    moveit_config = (
        MoveItConfigsBuilder("rm_75_description", package_name="rm_75_config")
        .robot_description(
            file_path="config/rm_75_description.urdf.xacro",
            mappings={"link7_type": "Link7_6f"},
        )
        .to_moveit_configs()
    )

    # Common launch configurations
    lc_use_sim_time = LaunchConfiguration("use_sim_time")
    lc_rviz_cfg = LaunchConfiguration("rviz_config")
    lc_pause = LaunchConfiguration("pause_gazebo")
    lc_world = LaunchConfiguration("world")
    lc_entity = LaunchConfiguration("entity_name")

    # ----------- Gazebo (classic) -----------
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
        ),
        launch_arguments={
            "pause": lc_pause,
            "world": lc_world,
        }.items(),
    )

    # ----------- Static TFs from SRDF virtual joints (if any) -----------
    static_tf_nodes = []
    name_counter = 0
    for _, srdf_xml in moveit_config.robot_description_semantic.items():
        srdf = SRDF.from_xml_string(srdf_xml)
        for vj in srdf.virtual_joints:
            static_tf_nodes.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"static_transform_publisher{name_counter}",
                    output="log",
                    arguments=["--frame-id", vj.parent_frame, "--child-frame-id", vj.child_link],
                    parameters=[{"use_sim_time": lc_use_sim_time}],
                )
            )
            name_counter += 1

    # ----------- Robot State Publisher & Joint State Publisher -----------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"publish_frequency": LaunchConfiguration("publish_frequency")},
            {"use_sim_time": lc_use_sim_time},
        ],
    )

    jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": lc_use_sim_time}],
    )

    # ----------- Spawn robot into Gazebo (from /robot_description) -----------
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", lc_entity],
        output="screen",
    )

    # ----------- ros2_control + controller spawners -----------
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,  # URDF must include ros2_control tags for Gazebo
            os.fspath(moveit_config.package_path / "config" / "ros2_controllers.yaml"),
            {"use_sim_time": lc_use_sim_time},
        ],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawn controllers listed in MoveIt controller manager config
    controller_names = moveit_config.trajectory_execution.get(
        "moveit_simple_controller_manager", {}
    ).get("controller_names", [])

    controller_spawners = [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[ctrl, "--controller-manager", "/controller_manager"],
            output="screen",
        )
        for ctrl in controller_names
    ]

    # ----------- move_group -----------
    move_group_params = [
        moveit_config.to_dict(),
        {
            "publish_robot_description_semantic": True,
            "allow_trajectory_execution": True,
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
            "monitor_dynamics": False,
            "use_sim_time": lc_use_sim_time,
        },
    ]

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
        # If you need GDB: prefix=['gdb', '-ex', 'run', '--args']
        # additional_env={"DISPLAY": ":0"},
    )

    # ----------- RViz (gets URDF + SRDF explicitly!) -----------
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", lc_rviz_cfg],
        parameters=[
            {"use_sim_time": lc_use_sim_time},
            moveit_config.robot_description,             
            moveit_config.robot_description_semantic,     
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )



    # ----------- Assemble LD -----------
    ld = LaunchDescription()
    for a in (use_rviz, use_sim_time, pause_gazebo, world, rviz_config, publish_frequency, entity_name, db):
        ld.add_action(a)

    ld.add_action(gazebo_launch)
    for n in static_tf_nodes:
        ld.add_action(n)
    ld.add_action(rsp)
    ld.add_action(jsp)
    ld.add_action(ros2_control_node)
    ld.add_action(spawn_entity)
    ld.add_action(jsb_spawner)
    for s in controller_spawners:
        ld.add_action(s)
    ld.add_action(move_group)
    ld.add_action(rviz)

    return ld
