# Makefile for TurtleBot3 examples
#
# Usage:
#   make <target> <arg-name>=<arg-value>
#
# Examples:
#   make term
#   make demo-world USE_GPU=true
#   make demo-behavior TARGET_COLOR=green BT_TYPE=queue ENABLE_VISION=true

# Command-line arguments
USE_GPU ?= false        # Use GPU devices (set to true if you have an NVIDIA GPU)
TARGET_COLOR ?= blue    # Target color for behavior tree demo (red | green | blue)
BT_TYPE ?= queue        # Behavior tree type (naive | queue)
ENABLE_VISION ?= true   # Enable vision in behaviors if true, else just do navigation

# Docker variables
IMAGE_NAME = turtlebot3
BASE_DOCKERFILE = docker/dockerfile_tb3_base
OVERLAY_DOCKERFILE = docker/dockerfile_tb3_overlay

# Set Docker volumes and environment variables
DOCKER_VOLUMES = \
	--volume="${PWD}/tb3_autonomy":"/overlay_ws/src/tb3_autonomy":rw \
	--volume="${PWD}/tb3_worlds":"/overlay_ws/src/tb3_worlds":rw \
	--volume="${PWD}/turtlebot3":"/overlay_ws/src/turtlebot3":rw \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
DOCKER_ENV_VARS = \
	--env="ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
	--env="NVIDIA_DRIVER_CAPABILITIES=all" \
	--env="DISPLAY" \
	--env="QT_X11_NO_MITSHM=1"
ifeq ("${USE_GPU}", "true")
DOCKER_GPU_ARGS = "--runtime nvidia --gpus all"
endif
DOCKER_ARGS = --ipc=host --net=host --privileged \
	${DOCKER_VOLUMES} ${DOCKER_ENV_VARS} ${DOCKER_GPU_VARS}

# Set ROS launch arguments for examples
LAUNCH_ARGS = \
	target_color:=${TARGET_COLOR} \
	tree_type:=${BT_TYPE} \
	enable_vision:=${ENABLE_VISION}


###########
#  SETUP  #
###########
# Build the base image (depends on core image build)
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
	@docker run -it \
		${DOCKER_ARGS} ${IMAGE_NAME}_overlay \
		bash

# Start basic simulation included with TurtleBot3 packages
.PHONY: sim
sim:
	@docker run -it \
		${DOCKER_ARGS} ${IMAGE_NAME}_overlay \
		ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Start Terminal for teleoperating the TurtleBot3
.PHONY: teleop
teleop:
	@docker run -it \
		${DOCKER_ARGS} ${IMAGE_NAME}_overlay \
		ros2 run turtlebot3_teleop teleop_keyboard

# Start our own simulation demo world
.PHONY: demo-world
demo-world:
	@docker run -it \
		${DOCKER_ARGS} ${IMAGE_NAME}_overlay \
		ros2 launch tb3_worlds tb3_demo_world.launch.py

# Start our own simulation demo behavior (Python or C++)
.PHONY: demo-behavior-py
demo-behavior-py:
	@docker run -it \
		${DOCKER_ARGS} ${IMAGE_NAME}_overlay \
		ros2 launch tb3_autonomy tb3_demo_behavior_py.launch.py ${LAUNCH_ARGS}

.PHONY: demo-behavior-cpp
demo-behavior-cpp:
	@docker run -it \
		${DOCKER_ARGS} ${IMAGE_NAME}_overlay \
		ros2 launch tb3_autonomy tb3_demo_behavior_cpp.launch.py ${LAUNCH_ARGS}
