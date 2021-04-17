# Docker arguments
IMAGE_NAME = turtlebot3
BASE_DOCKERFILE = ${PWD}/docker/dockerfile_base
OVERLAY_DOCKERFILE = ${PWD}/docker/dockerfile_overlay
DISPLAY ?= :0.0

# Set Docker volumes and environment variables
DOCKER_CODE_VOLUMES = \
	--volume="${PWD}/tb3_autonomy":"/overlay_ws/src/tb3_autonomy":rw \
	--volume="${PWD}/tb3_worlds":"/overlay_ws/src/tb3_worlds":rw
DOCKER_GUI_VOLUMES = \
	--volume="$(XAUTH):$(XAUTH)":rw \
	--volume="$(XSOCK)":$(XSOCK):rw \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
DOCKER_ENV_VARS = \
	--env="NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics" \
	--env="DISPLAY=$(DISPLAY)" \
	--env="QT_X11_NO_MITSHM=1" \
	--env="XAUTHORITY=$(XAUTH)" \
	--env="XPASSTHROUGH=$(XPASSTHROUGH)"
DOCKER_ARGS = ${DOCKER_CODE_VOLUMES} ${DOCKER_GUI_VOLUMES} ${DOCKER_ENV_VARS}


###########
#  SETUP  #
###########
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
	@docker run -it --gpus all --net=host --privileged \
		${DOCKER_ARGS} ${IMAGE_NAME}_overlay \
		bash

# Start basic simulation included with TurtleBot3 packages
.PHONY: sim
sim:
	@docker run -it --gpus all --net=host --privileged \
		${DOCKER_ARGS} ${IMAGE_NAME}_overlay \
		bash -it -c "roslaunch turtlebot3_gazebo turtlebot3_world.launch"

# Start Terminal for teleoperating the TurtleBot3
.PHONY: teleop
teleop:
	@docker run -it --gpus all --net=host --privileged \
		${DOCKER_ARGS} ${IMAGE_NAME}_overlay \
		bash -it -c "roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch"

# Start our own simulation demo
.PHONY: demo
demo:
	@docker run -it --gpus all --net=host --privileged \
		${DOCKER_ARGS} ${IMAGE_NAME}_overlay \
		bash -it -c "roslaunch tb3_autonomy tb3_demo.launch"