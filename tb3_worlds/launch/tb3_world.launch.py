from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    world = join(get_package_share_directory("tb3_worlds"),
                 "worlds", "sim_house.world")
    launch_file_dir = join(get_package_share_directory("turtlebot3_gazebo"),
                           "launch")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
            ),
            launch_arguments={"world": world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
            ),
        ),

        ExecuteProcess(
            cmd=["ros2", "param", "set", "/gazebo", "use_sim_time", use_sim_time],
            output="screen"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch_file_dir, "/robot_state_publisher.launch.py"]),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        ),
    ])
