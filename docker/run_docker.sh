# Sample script to run a command in a Docker container
#
# Usage Example:
# ./run_docker.sh turtlebot3_base "roslaunch turtlebot3_gazebo turtlebot3_world.launch"

# Define Docker volumes and environment variables
DOCKER_VOLUMES="
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
"
DOCKER_ENV_VARS="
--env="NVIDIA_DRIVER_CAPABILITIES=all" \
--env="DISPLAY=${DISPLAY}" \
--env="QT_X11_NO_MITSHM=1" \
"
DOCKER_ARGS=${DOCKER_VOLUMES}" "${DOCKER_ENV_VARS}

# Run the command
docker run -it --gpus all --net=host $DOCKER_ARGS $1 bash -it -c "$2"