
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")

    # --- MoveIt config (arm only) ---
    moveit_config = MoveItConfigsBuilder(
        "rm_description", package_name="rm_75_config"
    ).to_moveit_configs()

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}],
    )

    rviz = Node(
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        package="rviz2",
        executable="rviz2",
        arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
        parameters=[moveit_config.to_dict(), moveit_config.robot_description_kinematics],
        output="screen"
    )

    return LaunchDescription([
        use_rviz_arg,
        move_group,
        rviz,
    ])
