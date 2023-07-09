#!/bin/bash
# Sample script to run a command in a Docker container
#
# Usage Example:
# ./run_docker.sh turtlebot3_behavior:overlay "ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"

# Define Docker volumes and environment variables
DOCKER_VOLUMES="
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--volume="${XAUTHORITY:-$HOME/.Xauthority}:/root/.Xauthority" \
"
DOCKER_ENV_VARS="
--env="TURTLEBOT3_MODEL=waffle_pi" \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="NVIDIA_DRIVER_CAPABILITIES=all" \
"
DOCKER_ARGS=${DOCKER_VOLUMES}" "${DOCKER_ENV_VARS}

# Run the command
docker run -it --net=host --ipc=host --privileged ${DOCKER_ARGS} "$1" bash -c "$2"
