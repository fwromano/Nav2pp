from __future__ import annotations

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _remove_file_if_present(path: str):
    if os.path.exists(path):
        os.remove(path)
    return []


def generate_launch_description() -> LaunchDescription:
    bringup_dir = get_package_share_directory("nav2_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")
    sim_dir = get_package_share_directory("nav2_minimal_tb3_sim")
    repo_root = os.environ.get("NAV2PP_REPO_ROOT", "/work/Nav2++")
    custom_nav_launch = os.path.join(repo_root, "docker", "nav2pp-desktop", "launch", "nav2pp_navigation_launch.py")

    namespace = LaunchConfiguration("namespace")
    slam = LaunchConfiguration("slam")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_intra_process_comms = LaunchConfiguration("use_intra_process_comms")
    use_respawn = LaunchConfiguration("use_respawn")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_simulator = LaunchConfiguration("use_simulator")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")
    headless = LaunchConfiguration("headless")
    world = LaunchConfiguration("world")
    robot_name = LaunchConfiguration("robot_name")
    robot_sdf = LaunchConfiguration("robot_sdf")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    z_pose = LaunchConfiguration("z_pose")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    declare_namespace_cmd = DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace")
    declare_slam_cmd = DeclareLaunchArgument("slam", default_value="False", description="Whether run a SLAM")
    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map", default_value=os.path.join(bringup_dir, "maps", "tb3_sandbox.yaml")
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="True", description="Use simulation clock if true"
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart", default_value="True", description="Automatically startup the nav2 stack"
    )
    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition", default_value="False", description="Whether to use composed bringup"
    )
    declare_use_intra_process_comms_cmd = DeclareLaunchArgument(
        "use_intra_process_comms", default_value="False", description="Whether to use intra process communication"
    )
    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn", default_value="True", description="Whether to respawn if a node crashes"
    )
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RVIZ config file to use",
    )
    declare_use_simulator_cmd = DeclareLaunchArgument(
        "use_simulator", default_value="True", description="Whether to start the simulator"
    )
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub", default_value="True", description="Whether to start the robot state publisher"
    )
    declare_use_rviz_cmd = DeclareLaunchArgument("use_rviz", default_value="True", description="Whether to start RVIZ")
    declare_headless_cmd = DeclareLaunchArgument("headless", default_value="False", description="Whether to execute gzclient")
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(sim_dir, "worlds", "tb3_sandbox.sdf.xacro"),
        description="Full path to world model file to load",
    )
    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name", default_value="turtlebot3_waffle", description="name of the robot"
    )
    declare_robot_sdf_cmd = DeclareLaunchArgument(
        "robot_sdf",
        default_value=os.path.join(sim_dir, "urdf", "gz_waffle.sdf.xacro"),
        description="Full path to robot sdf file to spawn the robot in gazebo",
    )
    declare_x_pose_cmd = DeclareLaunchArgument("x_pose", default_value="-2.00")
    declare_y_pose_cmd = DeclareLaunchArgument("y_pose", default_value="-0.50")
    declare_z_pose_cmd = DeclareLaunchArgument("z_pose", default_value="0.01")
    declare_roll_cmd = DeclareLaunchArgument("roll", default_value="0.00")
    declare_pitch_cmd = DeclareLaunchArgument("pitch", default_value="0.00")
    declare_yaw_cmd = DeclareLaunchArgument("yaw", default_value="0.00")

    urdf = os.path.join(sim_dir, "urdf", "turtlebot3_waffle.urdf")
    with open(urdf, "r", encoding="utf-8") as infp:
        robot_description = infp.read()

    start_robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_description}],
        remappings=remappings,
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz_launch.py")),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "rviz_config": rviz_config_file,
        }.items(),
    )

    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "localization_launch.py")),
        launch_arguments={
            "namespace": namespace,
            "map": map_yaml_file,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": params_file,
            "use_composition": use_composition,
            "use_intra_process_comms": use_intra_process_comms,
            "use_respawn": use_respawn,
            "container_name": "nav2_container",
        }.items(),
    )

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(custom_nav_launch),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": params_file,
            "use_composition": use_composition,
            "use_intra_process_comms": use_intra_process_comms,
            "use_respawn": use_respawn,
            "container_name": "nav2_container",
        }.items(),
    )

    world_sdf = tempfile.mktemp(prefix="nav2pp_", suffix=".sdf")
    world_sdf_xacro = ExecuteProcess(cmd=["xacro", "-o", world_sdf, ["headless:=", headless], world])
    gazebo_server = ExecuteProcess(cmd=["gz", "sim", "-r", "-s", world_sdf], output="screen")
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-v4 -g "]}.items(),
    )
    gz_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_dir, "launch", "spawn_tb3.launch.py")),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "robot_name": robot_name,
            "robot_sdf": robot_sdf,
            "x_pose": x_pose,
            "y_pose": y_pose,
            "z_pose": z_pose,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
        }.items(),
    )
    remove_temp_sdf_file = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
                OpaqueFunction(function=lambda _: _remove_file_if_present(world_sdf))
            ]
        )
    )

    ld = LaunchDescription()
    for action in [
        declare_namespace_cmd,
        declare_slam_cmd,
        declare_map_yaml_cmd,
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_autostart_cmd,
        declare_use_composition_cmd,
        declare_use_intra_process_comms_cmd,
        declare_use_respawn_cmd,
        declare_rviz_config_file_cmd,
        declare_use_simulator_cmd,
        declare_use_robot_state_pub_cmd,
        declare_use_rviz_cmd,
        declare_headless_cmd,
        declare_world_cmd,
        declare_robot_name_cmd,
        declare_robot_sdf_cmd,
        declare_x_pose_cmd,
        declare_y_pose_cmd,
        declare_z_pose_cmd,
        declare_roll_cmd,
        declare_pitch_cmd,
        declare_yaw_cmd,
        world_sdf_xacro,
        remove_temp_sdf_file,
        gz_robot,
        gazebo_server,
        gazebo_client,
        start_robot_state_publisher_cmd,
        rviz_cmd,
        localization_cmd,
        navigation_cmd,
    ]:
        ld.add_action(action)
    return ld
