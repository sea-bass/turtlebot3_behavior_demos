# This file has been modified from
# https://github.com/ros-navigation/nav2_minimal_turtlebot_simulation/blob/091b5ff12436890a54de6325df3573ae110e912b/nav2_minimal_tb3_sim/launch/spawn_tb3.launch.py
# Modification:
#   - Add gz_bridge_config argument for specifying bridge yaml file
#   - Spawn Robot: Given the environment variable ("TURTLEBOT_MODEL"), choose either:
#       - Spawn turtlebot3 using xacro executable
#       - Spawn turtlebot4 using robot_description topic

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    EnvironmentVariable,
)
from launch.substitutions.command import Command
from launch.substitutions.find_executable import FindExecutable

from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    gz_mdl_dir = get_package_share_directory("nav2_minimal_tb3_sim")
    bringup_dir = get_package_share_directory("tb_worlds")

    namespace = LaunchConfiguration("namespace")
    robot_name = LaunchConfiguration("robot_name")
    robot_sdf = LaunchConfiguration("robot_sdf")
    turtlebot_model = LaunchConfiguration("turtlebot_model")
    gz_bridge_config = LaunchConfiguration("gz_bridge_config")
    pose = {
        "x": LaunchConfiguration("x_pose", default="0.0"),
        "y": LaunchConfiguration("y_pose", default="0.0"),
        "z": LaunchConfiguration("z_pose", default="0.01"),
        "R": LaunchConfiguration("roll", default="0.00"),
        "P": LaunchConfiguration("pitch", default="0.00"),
        "Y": LaunchConfiguration("yaw", default="0.00"),
    }

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        "robot_name", default_value="turtlebot3", description="name of the robot"
    )

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        "robot_sdf",
        default_value=os.path.join(bringup_dir, "urdf", "gz_waffle.sdf.xacro"),
        description="Full path to robot sdf file to spawn the robot in gazebo",
    )

    declare_turtlebot_model_cmd = DeclareLaunchArgument(
        "turtlebot_model",
        default_value=EnvironmentVariable("TURTLEBOT_MODEL", default_value="3"),
    )

    declare_gz_bridge_cmd = DeclareLaunchArgument(
        "gz_bridge_config",
        default_value=os.path.join(bringup_dir, "configs", "turtlebot3_bridge.yaml"),
        description="Full path to robot bridge configuration file",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=namespace,
        parameters=[
            {
                "config_file": gz_bridge_config,
                "expand_gz_topic_names": True,
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    spawn_tb3_model = Node(
        condition=IfCondition(PythonExpression([turtlebot_model, " == 3"])),
        package="ros_gz_sim",
        executable="create",
        output="screen",
        namespace=namespace,
        arguments=[
            "-name",
            robot_name,
            "-string",
            Command(
                [
                    FindExecutable(name="xacro"),
                    " ",
                    "namespace:=",
                    LaunchConfiguration("namespace"),
                    " ",
                    robot_sdf,
                ]
            ),
            "-x",
            pose["x"],
            "-y",
            pose["y"],
            "-z",
            pose["z"],
            "-R",
            pose["R"],
            "-P",
            pose["P"],
            "-Y",
            pose["Y"],
        ],
    )

    spawn_tb4_model = Node(
        condition=IfCondition(PythonExpression([turtlebot_model, " == 4"])),
        package="ros_gz_sim",
        executable="create",
        namespace=namespace,
        output="screen",
        arguments=[
            "-name",
            robot_name,
            "-topic",
            "robot_description",
            "-x",
            pose["x"],
            "-y",
            pose["y"],
            "-z",
            pose["z"],
            "-R",
            pose["R"],
            "-P",
            pose["P"],
            "-Y",
            pose["Y"],
        ],
        parameters=[{"use_sim_time": True}],
    )

    set_env_vars_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.join(gz_mdl_dir, "models")
    )
    set_env_vars_resources2 = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", str(Path(os.path.join(gz_mdl_dir)).parent.resolve())
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_turtlebot_model_cmd)
    ld.add_action(declare_gz_bridge_cmd)
    ld.add_action(set_env_vars_resources)
    ld.add_action(set_env_vars_resources2)

    ld.add_action(bridge)
    ld.add_action(spawn_tb3_model)
    ld.add_action(spawn_tb4_model)
    return ld
