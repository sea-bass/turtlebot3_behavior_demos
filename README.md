# docker-make-ros
Example repository for Docker + Make workflows in ROS based projects

In this example, we will set up a simulation environment for the [ROBOTIS TurtleBot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview) using Ubuntu 20.04 and ROS Noetic.

For more information, refer to the accompanying blog post (TODO).

By Sebastian Castro, 2021

---

## Setup
First, install Docker using [the official install guide](https://docs.docker.com/engine/install/ubuntu/).

To run Docker containers with graphics and GPU support, you will also need the [NVIDIA Container Toolkit](https://github.com/NVIDIA/nvidia-docker).

To use GUI based tools (e.g., RViz, Gazebo) inside Docker, there is additional setup required. The simplest way is to run the command below each time you log into your machine, but there is a more detailed walkthrough of options in the [ROS Wiki](http://wiki.ros.org/docker/Tutorials/GUI).

```
xhost +
```

---

## Usage
First, build the base and overlay Docker images. This will take a while.

```
make build
```

To enter a Terminal in the overlay container:

```
make term
```

To run a basic Gazebo simulation included with the standard TurtleBot3 packages:

```
make sim
```

To run our demo from the overlay workspace:

```
make demo
```
