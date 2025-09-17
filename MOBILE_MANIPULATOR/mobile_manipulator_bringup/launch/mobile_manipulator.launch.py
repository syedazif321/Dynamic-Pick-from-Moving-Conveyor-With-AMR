#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_desc = get_package_share_directory('mobile_manipulator_description')
    pkg_control = get_package_share_directory('mobile_manipulator_control')
    pkg_gazebo = get_package_share_directory('alphabot_gazebo')

    urdf_file = os.path.join(pkg_desc, "urdf", "mobile_manipulator.urdf.xacro")

    # Expand xacro → URDF
    robot_description_content = Command([FindExecutable(name="xacro"), " ", urdf_file])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # --- Gazebo ---
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',
             os.path.join(pkg_gazebo, 'worlds', 'no_roof_small_warehouse.world')],
        output='screen'
    )

    # --- robot_state_publisher ---
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
        output="screen"
    )

    # --- Spawn robot entity ---
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "mobile_manipulator"],
        output="screen"
    )

    # --- Controller Spawners (delayed so controller_manager is ready) ---
    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )
    diff_drive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_base", "--controller-manager", "/controller_manager"],
        output="screen"
    )
    slider = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["slider_position_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )
    arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rm_group_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    # Add delays to let gazebo_ros2_control start
    delayed_jsb = TimerAction(period=3.0, actions=[jsb])
    delayed_diff = TimerAction(period=4.0, actions=[diff_drive])
    delayed_slider = TimerAction(period=5.0, actions=[slider])
    delayed_arm = TimerAction(period=6.0, actions=[arm])

    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        delayed_jsb,
        delayed_diff,
        delayed_slider,
        delayed_arm
    ])
