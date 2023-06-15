from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    tb3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")

    tb3_world_dir = get_package_share_directory("tb3_worlds")
    default_map = join(tb3_world_dir, "maps", "sim_house_map.yaml")
    default_world = join(tb3_world_dir, "worlds", "sim_house.world")

    # Start Gazebo server and client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(gazebo_ros_dir, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": default_world}.items(),
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        )
    )

    # Start robot state publisher
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(tb3_gazebo_dir, "launch", "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # Spawn the turtlebot
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(tb3_gazebo_dir, "launch", "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    return LaunchDescription(
        [
            gzserver_cmd,
            gzclient_cmd,
            robot_state_publisher_cmd,
            spawn_turtlebot_cmd,
        ]
    )
