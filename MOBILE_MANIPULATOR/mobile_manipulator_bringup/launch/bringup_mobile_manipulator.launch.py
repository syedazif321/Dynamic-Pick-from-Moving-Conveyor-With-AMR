#!/usr/bin/env python3
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, FindExecutable, Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # --- Package paths ---
    pkg_desc = get_package_share_directory("mobile_manipulator_description")
    pkg_gazebo = get_package_share_directory("alphabot_gazebo")
    pkg_moveit = get_package_share_directory("rm_75_config")
    pkg_arm = get_package_share_directory("rm_description")
 
    srdf_file = os.path.join(pkg_moveit, "config", "rm_75_description.srdf")
    full_urdf_file = os.path.join(pkg_desc, "urdf", "mobile_manipulator.urdf.xacro")
    # Note: arm_urdf_file is defined but not used for MoveIt config when using MoveItConfigsBuilder
    arm_urdf_file = os.path.join(pkg_arm, "urdf", "rm_75_gazebo.urdf")
    
    # --- MoveIt Config Builder (Generate ALL MoveIt Parameters ONCE) ---
    moveit_config = MoveItConfigsBuilder(
        "rm_description", package_name="rm_75_config"
    ).to_moveit_configs()
    
    # Get the complete dictionary of parameters to pass to nodes
    moveit_params_dict = moveit_config.to_dict()

    # --- Launch arguments ---
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([pkg_gazebo, "worlds", "no_roof_small_warehouse.world"]),
        description="SDF world file"
    )
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    entity_arg = DeclareLaunchArgument("entity", default_value="mobile_manipulator")
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="true")

    # --- Robot description for Gazebo (Full Robot) ---
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", full_urdf_file]), value_type=None
        )
    }

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

    # --- Event chaining for controllers (Base setup: Spawner -> JSB -> Arm) ---
    load_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawner, on_exit=[jsb_spawner])
    )
    load_arm = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[arm_spawner])
    )

    # --- Vision node ---
    vision_node = Node(
        package='pipeline_manipulator',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    # --- MoveGroup Node (MoveIt Planning Engine) ---
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_params_dict, 
            {"use_sim_time": True}
        ],
    )
    
    # --- Dynamic Pick Controller Node (receives full MoveIt Config) ---
    dynamic_pick_controller_node = Node(
        package='pipeline_manipulator',
        executable='dynamic_pick_controller', 
        name='dynamic_pick_controller',      
        output='screen',
        parameters=[
            moveit_params_dict, 
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )

    # --- RViz Node (Visualization) ---
    rviz = Node(
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        package="rviz2",
        executable="rviz2",
        arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
        parameters=[
            moveit_params_dict,
            {"use_sim_time": True}
        ],
        output="screen"
    )
    
    # --- CRITICAL FIX: Launch MoveGroup and Controller in Sequence ---

    # 1. Launch MoveGroup node after the arm controllers are activated (arm_spawner exits)
    load_move_group = RegisterEventHandler(
        OnProcessExit(target_action=arm_spawner, on_exit=[move_group])
    )

    # 2. FIX: Launch your custom controller (dynamic_pick_controller_node) AFTER a delay.
    # We use a 5-second TimerAction to ensure MoveGroup (launched above) has time to initialize 
    # and open all its services before the controller tries to connect.
    delay_pick_controller = TimerAction(
        period=15.0,  
        actions=[dynamic_pick_controller_node],
    )


    return LaunchDescription([
        world_arg, 
        use_sim_time_arg, 
        entity_arg, 
        use_rviz_arg,
        
        # Gazebo and TF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_gazebo, "launch", "gazebo.launch.py")),
            launch_arguments={"world": LaunchConfiguration("world")}.items(),
        ),
        rsp,
        spawner,
        
        # Controller Spawners (Sequential)
        load_jsb,
        load_arm,
        
        # Main nodes
        vision_node,
        
        # CRITICAL: Launch MoveGroup (via event from arm_spawner)
        load_move_group, 
        
        # CRITICAL FIX: Launch your controller after a delay
        delay_pick_controller, 
        
        rviz
    ])