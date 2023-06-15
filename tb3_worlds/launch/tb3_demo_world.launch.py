from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from os.path import join


def generate_launch_description():
    tb3_nav2_dir = get_package_share_directory("turtlebot3_navigation2")
    tb3_world_dir = get_package_share_directory("tb3_worlds")

    # Spawn the world and robot
    spawn_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(tb3_world_dir, "launch", "tb3_world.launch.py")
        ),
        launch_arguments={
            "x_pose": LaunchConfiguration("x_pose", default=0.0),
            "y_pose": LaunchConfiguration("y_pose", default=0.0),
        }.items(),
    )
    # For some reason, there is an error with starting both spawn world and nav
    # in this launch file without this delay ???
    spawn_world_delayed = TimerAction(period=3.0, actions=[spawn_world])

    # Start navigation stack
    default_map = join(tb3_world_dir, "maps", "sim_house_map.yaml")
    nav_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(tb3_nav2_dir, "launch", "navigation2.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time", default="true"),
            "map": LaunchConfiguration("map", default=default_map),
        }.items(),
    )

    # Set AMCL initial pose
    amcl_init_pose = Node(
        package="tb3_worlds",
        executable="set_init_amcl_pose.py",
        name="init_pose_publisher",
    )

    # Spawn blocks
    spawn_blocks = Node(
        package="tb3_worlds",
        executable="block_spawner.py",
        name="block_spawner",
        parameters=[
            {"location_file": join(tb3_world_dir, "maps", "sim_house_locations.yaml")}
        ],
    )

    return LaunchDescription(
        [spawn_world_delayed, nav_stack, amcl_init_pose, spawn_blocks]
    )
