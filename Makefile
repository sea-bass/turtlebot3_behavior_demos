# Docker arguments
IMAGE_NAME = turtlebot3
DISPLAY ?= :0.0
BASE_DOCKERFILE = ${PWD}/docker/dockerfile_base
OVERLAY_DOCKERFILE = ${PWD}/docker/dockerfile_overlay

# Set Docker volumes and environment variables
DOCKER_VOLUMES = \
	--volume="$(XAUTH):$(XAUTH)":rw \
	--volume="$(XSOCK)":$(XSOCK):rw \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--volume="$(DOCKER_FILE_DIR)/scripts":"/scripts":rw
DOCKER_ENV_VARS = \
	--env="NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics" \
	--env="DISPLAY=$(DISPLAY)" \
	--env="QT_X11_NO_MITSHM=1" \
	--env="XAUTHORITY=$(XAUTH)" \
	--env="XPASSTHROUGH=$(XPASSTHROUGH)"


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
	@docker kill $(shell docker ps -q --filter ancestor=${IMAGE_NAME})


###########
#  TASKS  #
###########
# Start a terminal
.PHONY: term
term:
	@docker run -it --gpus all --net=host --ipc host --privileged \
		${DOCKER_VOLUMES} ${DOCKER_ENV_VARS} ${IMAGE_NAME}_overlay \
		bash

# Start basic simulation
.PHONY: sim
sim:
	@docker run -it --gpus all --net=host --ipc host --privileged \
		${DOCKER_VOLUMES} ${DOCKER_ENV_VARS} ${IMAGE_NAME}_overlay \
		bash -it -c "roslaunch turtlebot3_gazebo turtlebot3_world.launch"

# TODO: Start launch file from overlay workspace