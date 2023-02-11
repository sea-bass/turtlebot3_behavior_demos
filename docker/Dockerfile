########################################
# Base Image for TurtleBot3 Simulation #
########################################
FROM ros:humble AS base
SHELL ["/bin/bash", "-c"]

# Install basic apt packages
RUN apt-get update && apt-get install -y --no-install-recommends \
 git libcanberra-gtk-module libcanberra-gtk3-module libqt5svg5-dev \
 python3-pip python3-opencv python3-tk python3-pyqt5.qtwebengine

# Install additional Python modules
RUN pip3 install matplotlib transforms3d

# Install additional ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
 ros-humble-cartographer \
 ros-humble-cartographer-ros \
 ros-humble-gazebo-ros \
 ros-humble-navigation2 \
 ros-humble-nav2-bringup \
 ros-humble-rmw-cyclonedds-cpp

# Use Cyclone DDS as middleware
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Downgrade mesa packages because of Ubuntu bug
# https://bugs.launchpad.net/ubuntu/+source/mesa/+bug/2004649
RUN apt-get update && apt-get install -q -y --allow-downgrades --no-install-recommends \
    libegl-mesa0=22.0.1-1ubuntu2 \
    libgbm1=22.0.1-1ubuntu2 \
    libgbm-dev=22.0.1-1ubuntu2 \
    libgl1-mesa-dri=22.0.1-1ubuntu2 \
    libglapi-mesa=22.0.1-1ubuntu2 \
    libglx-mesa0=22.0.1-1ubuntu2

# Create Colcon workspace with external dependencies
RUN mkdir -p /turtlebot3_ws/src
WORKDIR /turtlebot3_ws/src
COPY dependencies.repos ./
RUN vcs import < dependencies.repos

# Build the base Colcon workspace, installing any extra missed dependencies.
WORKDIR /turtlebot3_ws
RUN source /opt/ros/humble/setup.bash \
 && apt-get update -y \
 && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
RUN source /opt/ros/humble/setup.bash \
 && colcon build --symlink-install

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE 1

# Set up the entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

#######################################################
# Overlay Development Image for TurtleBot3 Simulation #
#######################################################
FROM base AS overlay

# Create an overlay Colcon workspace
RUN mkdir -p /overlay_ws/src
WORKDIR /overlay_ws
COPY ./tb3_autonomy/ ./src/tb3_autonomy/
COPY ./tb3_worlds/ ./src/tb3_worlds/
RUN source /turtlebot3_ws/install/setup.bash \
 && colcon build --symlink-install

# Set up the entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]
