#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ---------------- Args ----------------
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    ns_arg = DeclareLaunchArgument("namespace", default_value="", description="ROS namespace")

    use_gz_cam_arg = DeclareLaunchArgument("use_gazebo_camera", default_value="false")
    cmd_vel_arg    = DeclareLaunchArgument("cmd_vel_topic",     default_value="/cmd_vel")
    odom_arg       = DeclareLaunchArgument("odom_topic",        default_value="/odom")
    scan_arg       = DeclareLaunchArgument("scan_topic",        default_value="/scan")

    rviz_cfg_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([
            get_package_share_directory("alphabot_bringup"),
            "config", "alphabot.rviz"
        ]),
        description="RViz config file"
    )

    # ---------------- Launch Configs ----------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    ns = LaunchConfiguration("namespace")
    use_gazebo_camera = LaunchConfiguration("use_gazebo_camera")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    rviz_config = LaunchConfiguration("rviz_config")

    # ---------------- Robot description from Xacro ----------------
    urdf_file = PathJoinSubstitution([
        get_package_share_directory("alphabot_description"),
        "urdf", "alphabot.urdf.xacro",
    ])

    robot_description_cmd = Command([
        FindExecutable(name="xacro"), " ",
        urdf_file, " ",
        "use_gazebo_camera:=", use_gazebo_camera, " ",
        "cmd_vel_topic:=",     cmd_vel_topic,     " ",
        "odom_topic:=",        odom_topic,        " ",
        "scan_topic:=",        scan_topic,
    ])
    robot_description = ParameterValue(robot_description_cmd, value_type=str)

    # ---------------- Robot State Publisher ----------------
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns,
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time
        }],
        output="screen",
    )

    # ---------------- RViz2 ----------------
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        namespace=ns,
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription([
        use_sim_time_arg, ns_arg,
        use_gz_cam_arg, cmd_vel_arg, odom_arg, scan_arg,
        rviz_cfg_arg,
        rsp, rviz,
    ])
