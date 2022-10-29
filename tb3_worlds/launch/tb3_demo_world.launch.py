from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    tb3_world_dir = get_package_share_directory("tb3_worlds")
    default_map = join(tb3_world_dir, "maps", "sim_house_map.yaml")
    default_world = join(tb3_world_dir, "worlds", "sim_house.world")

    return LaunchDescription([
        # Spawn the world and nav stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                join(nav2_bringup_dir, "launch", "tb3_simulation_launch.py")
            ),
            launch_arguments={
                "headless": LaunchConfiguration("headless", default=False),
                "use_sim_time": LaunchConfiguration("use_sim_time", default="true"),
                "world": LaunchConfiguration("world", default=default_world),
                "map": LaunchConfiguration("map", default=default_map),
                "x_pose": LaunchConfiguration("x_pose", default=0.0),
                "y_pose": LaunchConfiguration("y_pose", default=0.0),
            }.items()
        ),

        # Set AMCL initial pose
        Node(
            package="tb3_worlds",
            executable="set_init_amcl_pose.py",
            name="init_pose_publisher",
        ),

        # Spawn blocks
        Node(
            package="tb3_worlds",
            executable="block_spawner.py",
            name="block_spawner",
            parameters=[{
                "location_file": join(tb3_world_dir, "maps", "sim_house_locations.yaml")
            }]
        ),

    ])
