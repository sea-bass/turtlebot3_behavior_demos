#!/usr/bin/env python3

"""
Spawns blocks randomly given a list of possible locations.

This node spawns one block each (red, blue, and green), so at least 
3 locations are required in the current implementation.
"""

import os
import math
import yaml
import random
import transforms3d
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity


# Define some parameters
model_dir = os.path.join(get_package_share_directory("tb3_worlds"), "models")

class BlockSpawner(Node):
    def __init__(self):
        super().__init__("block_spawner")
        self.cli = self.create_client(SpawnEntity, "/spawn_entity")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service...")
        self.get_logger().info("Started block spawner service")

        self.declare_parameter("location_file")


    def spawn_blocks(self):
        # Parse locations YAML file
        location_file = self.get_parameter("location_file").value
        self.get_logger().info(f"Using location file: {location_file}")
        with open(location_file, "r") as f:
            locations = yaml.load(f, Loader=yaml.FullLoader)
        
        # Distance from navigation waypoint to spawn object
        block_spawn_offset = 0.6

        # Spawn blocks in random locations
        model_names = ["red_block", "green_block", "blue_block"]
        sampled_locs = random.sample(list(locations.keys()), len(model_names))
        for mdl, loc in zip(model_names, sampled_locs):
            x, y, theta = locations[loc]
            x += block_spawn_offset * math.cos(theta)
            y += block_spawn_offset * math.sin(theta)
            self.spawn_model(mdl, x, y, theta)

    def spawn_model(self, model_name, x, y, theta):
        """ Spawns a model in Gazebo given a position """
        model_file = os.path.join(model_dir, model_name, "model.sdf")
        with open(model_file, "r") as f:
            model_xml = f.read()
        
        req = SpawnEntity.Request()
        req.name = model_name
        req.xml = model_xml
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        quat = transforms3d.euler.euler2quat(0, 0, theta)
        req.initial_pose.orientation.w = quat[0]
        req.initial_pose.orientation.x = quat[1]
        req.initial_pose.orientation.y = quat[2]
        req.initial_pose.orientation.z = quat[3]
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


if __name__=="__main__":
    rclpy.init()

    spawner = BlockSpawner()
    spawner.spawn_blocks()

    rclpy.spin(spawner)
    rclpy.shutdown()

