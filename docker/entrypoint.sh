#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash
echo "Sourced ROS 2 Humble"

# Source the base workspace, if built
if [ -f /turtlebot3_ws/install/setup.bash ]
then
  echo "source /turtlebot3_ws/install/setup.bash" >> ~/.bashrc
  source /turtlebot3_ws/install/setup.bash
  export TURTLEBOT3_MODEL=waffle_pi
  export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models
  echo "Sourced TurtleBot3 base workspace"
fi

# Source the overlay workspace, if built
if [ -f /overlay_ws/install/setup.bash ]
then
  echo "source /overlay_ws/install/setup.bash" >> ~/.bashrc
  source /overlay_ws/install/setup.bash
  export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix tb3_worlds)/share/tb3_worlds/models
  echo "Sourced autonomy overlay workspace"
fi

# Execute the command passed into this entrypoint
exec "$@"
