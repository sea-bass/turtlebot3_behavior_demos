#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers

# Source ROS and Colcon workspaces
source /opt/ros/galactic/setup.bash
echo "Sourced ROS2 Galactic"
if [ -f /turtlebot3_ws/install/setup.bash ]
then
  echo "source /turtlebot3_ws/install/setup.bash" >> ~/.bashrc
  source /turtlebot3_ws/install/setup.bash
  echo "Sourced TurtleBot3 base workspace"
fi
if [ -f /overlay_ws/install/setup.bash ]
then
  echo "source /overlay_ws/install/setup.bash" >> ~/.bashrc
  source /overlay_ws/install/setup.bash
  echo "Sourced autonomy overlay workspace"
fi

# Set environment variables
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix tb3_worlds)/share/tb3_worlds/models

# Execute the command passed into this entrypoint
exec "$@"
