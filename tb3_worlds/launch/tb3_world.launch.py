# This is a modified copy of https://github.com/ros-navigation/navigation2/blob/d6b02ac407a26d9394a23f76620f25f2274c8c37/nav2_bringup/launch/tb3_simulation_launch.py
# Launches robot_state_publisher and gazebo with world argument
import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    EnvironmentVariable,
    PythonExpression,
    Command,
)
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("tb3_worlds")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    turtlebot_model = LaunchConfiguration("turtlebot_model")

    # Launch configuration variables specific to simulation
    world = LaunchConfiguration("world")
    pose = {
        "x": LaunchConfiguration("x_pose", default="0.0"),
        "y": LaunchConfiguration("y_pose", default="0.0"),
        "z": LaunchConfiguration("z_pose", default="0.01"),
        "R": LaunchConfiguration("roll", default="0.00"),
        "P": LaunchConfiguration("pitch", default="0.00"),
        "Y": LaunchConfiguration("yaw", default="0.00"),
    }
    robot_name = LaunchConfiguration("robot_name")
    robot_sdf = LaunchConfiguration("robot_sdf")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(bringup_dir, "worlds", "sim_house.sdf.xacro"),
        description="Full path to world model file to load",
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name", default_value="turtlebot", description="name of the robot"
    )

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        "robot_sdf",
        default_value=os.path.join(bringup_dir, "urdf", "gz_waffle.sdf.xacro"),
        description="Full path to robot sdf file to spawn the robot in gazebo",
    )

    turtlebot_model_cmd = DeclareLaunchArgument(
        "turtlebot_model",
        default_value=EnvironmentVariable("TURTLEBOT3_MODEL", default_value="4"),
    )

    turtlebot_model_os_value = os.getenv("TURTLEBOT3_MODEL", "4")

    if turtlebot_model_os_value == "3":
        urdf = os.path.join(
            get_package_share_directory("nav2_minimal_tb3_sim"),
            "urdf",
            "turtlebot3_waffle.urdf",
        )
        with open(urdf, "r") as infp:
            robot_description = infp.read()
    else:
        robot_description = Command(
            [
                "xacro",
                " ",
                os.path.join(
                    get_package_share_directory("nav2_minimal_tb4_description"),
                    "urdf",
                    "standard",
                    "turtlebot4.urdf.xacro",
                ),
            ]
        )

    robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time, "robot_description": robot_description}
        ],
        remappings=remappings,
    )

    # The SDF file for the world is a xacro file because we wanted to
    # conditionally load the SceneBroadcaster plugin based on whether we're
    # running in headless mode. But currently, the Gazebo command line doesn't
    # take SDF strings for worlds, so the output of xacro needs to be saved into
    # a temporary file and passed to Gazebo.
    world_sdf = tempfile.mktemp(prefix="tb3_", suffix=".sdf")
    world_sdf_xacro = ExecuteProcess(cmd=["xacro", "-o", world_sdf, world])
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_sdf],
        output="screen",
    )

    remove_temp_sdf_file = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[OpaqueFunction(function=lambda _: os.remove(world_sdf))]
        )
    )

    gz_tb3_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "spawn_tb3.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "robot_name": "turtlebot3",
            "robot_sdf": robot_sdf,
            "x_pose": pose["x"],
            "y_pose": pose["y"],
            "z_pose": pose["z"],
            "roll": pose["R"],
            "pitch": pose["P"],
            "yaw": pose["Y"],
        }.items(),
        condition=IfCondition(PythonExpression([turtlebot_model, " == 3"])),
    )

    gz_tb4_spawner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_minimal_tb4_sim"),
                "launch",
                "spawn_tb4.launch.py",
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "robot_name": "turtlebot4",
            "x_pose": pose["x"],
            "y_pose": pose["y"],
            "z_pose": pose["z"],
            "roll": pose["R"],
            "pitch": pose["P"],
            "yaw": pose["Y"],
        }.items(),
        condition=IfCondition(PythonExpression([turtlebot_model, " == 4"])),
    )

    # Create the launch description
    return LaunchDescription(
        [
            declare_namespace_cmd,
            declare_use_sim_time_cmd,
            declare_world_cmd,
            declare_robot_name_cmd,
            declare_robot_sdf_cmd,
            turtlebot_model_cmd,
            world_sdf_xacro,
            remove_temp_sdf_file,
            gz_tb3_spawner,
            gz_tb4_spawner,
            gazebo,
            robot_state_publisher_cmd,
        ]
    )
