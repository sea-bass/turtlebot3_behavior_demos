# Makefile for TurtleBot3 examples
#
# Usage:
#   make <target> <arg-name>=<arg-value>
#
# Examples:
#   make term
#   make demo-world USE_GPU=true
#   make demo-behavior USE_GPU=false TARGET_COLOR=green BT_TYPE=queue

# Command-line arguments
TARGET_COLOR ?= blue    # Target color for behavior tree demo (red | green | blue)
USE_GPU ?= false        # Use GPU devices (set to true if you have an NVIDIA GPU)
BT_TYPE ?= queue        # Behavior tree type (naive | queue)

# Docker variables
IMAGE_NAME = turtlebot3
CORE_DOCKERFILE = ${PWD}/docker/dockerfile_nvidia_ros
BASE_DOCKERFILE = ${PWD}/docker/dockerfile_tb3_base
OVERLAY_DOCKERFILE = ${PWD}/docker/dockerfile_tb3_overlay

# Set Docker volumes and environment variables
DOCKER_VOLUMES = \
	--volume="${PWD}/tb3_autonomy":"/overlay_ws/src/tb3_autonomy":rw \
	--volume="${PWD}/tb3_worlds":"/overlay_ws/src/tb3_worlds":rw \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
DOCKER_ENV_VARS = \
	--env="NVIDIA_DRIVER_CAPABILITIES=all" \
	--env="DISPLAY" \
	--env="QT_X11_NO_MITSHM=1"
ifeq ("${USE_GPU}", "true")
DOCKER_GPU_ARGS = "--gpus all"
endif
DOCKER_ARGS = ${DOCKER_VOLUMES} ${DOCKER_ENV_VARS} ${DOCKER_GPU_VARS}


###########
#  SETUP  #
###########
# Build the core image
.PHONY: build-core
build-core:
	@docker build -f ${CORE_DOCKERFILE} -t nvidia_ros .

# Build the base image
.PHONY: build-base
build-base:
	@docker build -f ${BASE_DOCKERFILE} -t ${IMAGE_NAME}_base .

# Build the overlay image (depends on base image build)
.PHONY: build
build: build-base
	@docker build -f ${OVERLAY_DOCKERFILE} -t ${IMAGE_NAME}_overlay .

# Kill any running Docker containers
.PHONY: kill
kill:
	@echo "Closing all running Docker containers:"
	@docker kill $(shell docker ps -q --filter ancestor=${IMAGE_NAME}_overlay)


###########
#  TASKS  #
###########
# Start a terminal inside the Docker container
.PHONY: term
term:
	@docker run -it --net=host \
		${DOCKER_ARGS} ${IMAGE_NAME}_overlay \
		bash

# Start basic simulation included with TurtleBot3 packages
.PHONY: sim
sim:
	@docker run -it --net=host \
		${DOCKER_ARGS} ${IMAGE_NAME}_overlay \
		roslaunch turtlebot3_gazebo turtlebot3_world.launch

# Start Terminal for teleoperating the TurtleBot3
.PHONY: teleop
teleop:
	@docker run -it --net=host \
		${DOCKER_ARGS} ${IMAGE_NAME}_overlay \
		roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

# Start our own simulation demo world
.PHONY: demo-world
demo-world:
	@docker run -it --net=host \
		${DOCKER_ARGS} ${IMAGE_NAME}_overlay \
		roslaunch tb3_worlds tb3_demo_world.launch

# Start our own simulation demo behavior
.PHONY: demo-behavior
demo-behavior:
	@docker run -it --net=host \
		${DOCKER_ARGS} ${IMAGE_NAME}_overlay \
		roslaunch tb3_autonomy tb3_demo_behavior_py.launch \
		target_color:=${TARGET_COLOR} behavior_tree_type:=${BT_TYPE}

.PHONY: demo-behavior-cpp
demo-behavior-cpp:
	@docker run -it --net=host \
		${DOCKER_ARGS} ${IMAGE_NAME}_overlay \
		roslaunch tb3_autonomy tb3_demo_behavior_cpp.launch \
		target_color:=${TARGET_COLOR} behavior_tree_type:=${BT_TYPE}