from ast import arguments
from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_tb3_worlds = get_package_share_directory("tb3_worlds")
    default_world_dir = join(pkg_tb3_worlds, "maps", "sim_house_locations.yaml")

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            "location_file",
            default_value=TextSubstitution(text=default_world_dir),
            description="YAML file name containing map locations in the world."
        ),
        DeclareLaunchArgument(
            "target_color",
            default_value=TextSubstitution(text="blue"),
            description="Target object color (red, green, or blue)"
        ),
        DeclareLaunchArgument(
            "tree_type",
            default_value=TextSubstitution(text="queue"),
            description="Behavior tree type (naive or queue)"
        ),
        DeclareLaunchArgument(
            "enable_vision",
            default_value=TextSubstitution(text="True"),
            description="Enable vision behaviors. If false, do navigation only."
        ),
        # Start autonomy stack
        Node(
            package="tb3_autonomy",
            executable="autonomy_node_cpp",
            name="autonomy_node_cpp",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "location_file": LaunchConfiguration("location_file"),
                "target_color": LaunchConfiguration("target_color"),
                "tree_type": LaunchConfiguration("tree_type"),
                "enable_vision": LaunchConfiguration("enable_vision")
            }]
        ),
        # Behavior tree visualization
        ExecuteProcess(
            cmd=["ros2", "run", "groot", "Groot", "--mode", "monitor",
                 "--publisher_port", "1668", "--server_port", "1669",
                 "--autoconnect"]
        ),
    ])
