from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from os import environ
from os.path import join


# Change this to your Groot2 executable path.
# This currently defaults to a Groot2 AppImage file in your home directory.
groot2_executable = join(environ.get("HOME", "/"), "Groot2.AppImage")


def get_autonomy_and_visualization_nodes(context, *args, **kwargs):
    # Unpack arguments
    tree_type = LaunchConfiguration("tree_type").perform(context)
    enable_vision = (
        LaunchConfiguration("enable_vision").perform(context).lower() == "true"
    )

    prefix = "nav_" if not enable_vision else ""
    xml_file_name = f"{prefix}tree_{tree_type}.xml"
    print(f"\nUsing Behavior tree file: {xml_file_name}\n")

    pkg_tb3_autonomy = get_package_share_directory("tb3_autonomy")
    xml_file_path = join(pkg_tb3_autonomy, "bt_xml", xml_file_name)

    return [
        # Main autonomy node.
        Node(
            package="tb3_autonomy",
            executable="autonomy_node_cpp",
            name="autonomy_node_cpp",
            output="screen",
            emulate_tty=True,
            parameters=[
                {
                    "location_file": LaunchConfiguration("location_file"),
                    "target_color": (
                        LaunchConfiguration("target_color") if enable_vision else ""
                    ),
                    "tree_xml_file": xml_file_path,
                }
            ],
        ),
        # Behavior tree visualization with Groot2.
        ExecuteProcess(
            cmd=[groot2_executable, "--nosplash", "true", "--file", xml_file_path]
        ),
    ]


def generate_launch_description():
    pkg_tb3_worlds = get_package_share_directory("tb3_worlds")
    default_world_dir = join(pkg_tb3_worlds, "maps", "sim_house_locations.yaml")

    return LaunchDescription(
        [
            # Arguments
            DeclareLaunchArgument(
                "location_file",
                default_value=TextSubstitution(text=default_world_dir),
                description="YAML file name containing map locations in the world.",
            ),
            DeclareLaunchArgument(
                "target_color",
                default_value=TextSubstitution(text="blue"),
                description="Target object color (red, green, or blue)",
            ),
            DeclareLaunchArgument(
                "tree_type",
                default_value=TextSubstitution(text="queue"),
                description="Behavior tree type (naive or queue)",
            ),
            DeclareLaunchArgument(
                "enable_vision",
                default_value=TextSubstitution(text="True"),
                description="Enable vision behaviors. If false, do navigation only.",
            ),
            # Autonomy node and behavior tree visualization nodes
            OpaqueFunction(function=get_autonomy_and_visualization_nodes),
        ]
    )
