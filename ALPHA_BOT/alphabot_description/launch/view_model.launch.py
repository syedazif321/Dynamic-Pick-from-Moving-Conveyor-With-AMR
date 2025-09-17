from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ---------------- Args ----------------
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([
            get_package_share_directory("alphabot_gazebo"),
            "worlds", "empty.world"
        ]),
        description="SDF world file"
    )
    entity_arg = DeclareLaunchArgument("entity", default_value="alphabot")
    x_arg = DeclareLaunchArgument("x", default_value="0.0")
    y_arg = DeclareLaunchArgument("y", default_value="0.0")
    z_arg = DeclareLaunchArgument("z", default_value="0.1")
    yaw_arg = DeclareLaunchArgument("yaw", default_value="0.0")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    ns_arg = DeclareLaunchArgument("namespace", default_value="", description="ROS namespace")

    # Xacro/exposed robot knobs
    use_gz_cam_arg = DeclareLaunchArgument("use_gazebo_camera", default_value="false")
    cmd_vel_arg = DeclareLaunchArgument("cmd_vel_topic", default_value="/amazon_robot/cmd_vel")
    odom_arg    = DeclareLaunchArgument("odom_topic",     default_value="/amazon_robot/odom")
    scan_arg    = DeclareLaunchArgument("scan_topic",     default_value="/scan")

    # Optional RViz
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="false")
    rviz_cfg_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution([
            get_package_share_directory("alphabot_bringup"),
            "rviz", "alphabot.rviz"
        ]),
        description="RViz config (ignored if missing)"
    )

    # ---------------- LCs ----------------
    world = LaunchConfiguration("world")
    entity = LaunchConfiguration("entity")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    use_sim_time = LaunchConfiguration("use_sim_time")
    ns = LaunchConfiguration("namespace")

    use_gazebo_camera = LaunchConfiguration("use_gazebo_camera")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    scan_topic = LaunchConfiguration("scan_topic")

    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    # ---------------- Robot description from Xacro ----------------
    urdf_file = PathJoinSubstitution([
        get_package_share_directory("alphabot_description"),
        "urdf", "alphabot.urdf.xacro",
    ])

    # Important: keep literal spaces between tokens when using Command
    robot_description_cmd = Command([
        FindExecutable(name="xacro"), " ",
        urdf_file, " ",
        "use_gazebo_camera:=", use_gazebo_camera, " ",
        "cmd_vel_topic:=",     cmd_vel_topic,     " ",
        "odom_topic:=",        odom_topic,        " ",
        "scan_topic:=",        scan_topic
    ])
    robot_description = ParameterValue(robot_description_cmd, value_type=str)

    # ---------------- Gazebo (Classic) server+client ----------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("alphabot_gazebo"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

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

    # ---------------- Spawn entity in Gazebo ----------------
    spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        namespace=ns,
        arguments=[
            "-entity", entity,
            "-topic", "robot_description",
            "-x", x, "-y", y, "-z", z, "-Y", yaw
        ],
        output="screen",
    )

    # ---------------- Optional RViz2 ----------------
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        namespace=ns,
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription([
        # args
        world_arg, entity_arg, x_arg, y_arg, z_arg, yaw_arg, use_sim_time_arg, ns_arg,
        use_gz_cam_arg, cmd_vel_arg, odom_arg, scan_arg,
        use_rviz_arg, rviz_cfg_arg,
        # nodes
        gazebo, rsp, spawner, rviz,
    ])
