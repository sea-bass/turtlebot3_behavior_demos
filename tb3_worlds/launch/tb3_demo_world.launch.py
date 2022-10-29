from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    tb3_nav2_dir = get_package_share_directory("turtlebot3_navigation2")
    pkg_tb3_worlds = get_package_share_directory("tb3_worlds")

    default_map = join(pkg_tb3_worlds, "maps", "sim_house_map.yaml")
    print(f"Using map file: {default_map}")

    return LaunchDescription([
        # Spawn the world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                join(pkg_tb3_worlds, "launch", "tb3_world.launch.py")
            ),
        ),
        # Start navigation stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                join(tb3_nav2_dir, "launch", "navigation2.launch.py")
            ),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time", default="true"),
                "map": LaunchConfiguration("map", default=default_map)
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
                "location_file": join(pkg_tb3_worlds, "maps", "sim_house_locations.yaml")
            }]
        )
    ])
