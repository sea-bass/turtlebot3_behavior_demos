#!/usr/bin/env python3

"""
Spawns blocks randomly given a world
"""

import os
import tf
import math
import yaml
import rospy
import random
import rospkg
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest

# Define some parameters
pkg_dir = rospkg.RosPack().get_path("tb3_worlds")

def spawn_model(model_name, x, y, theta):
    """ Spawns a model in Gazebo given a position """
    model_file = os.path.join(pkg_dir, "models", model_name, "model.sdf")
    with open(model_file, "r") as f:
        model_xml = f.read()
    
    req = SpawnModelRequest()
    req.model_name = model_name
    req.model_xml = model_xml
    req.initial_pose.position.x = x
    req.initial_pose.position.y = y
    quat = tf.transformations.quaternion_from_euler(0, 0, theta)
    req.initial_pose.orientation.x = quat[0]
    req.initial_pose.orientation.y = quat[1]
    req.initial_pose.orientation.z = quat[2]
    req.initial_pose.orientation.w = quat[3]
    resp = spawner(req)

    return resp.success

if __name__=="__main__":
    # Start ROS node and Gazebo model spawner service client
    rospy.init_node("block_spawner")
    spawner = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    spawner.wait_for_service()

    # Parse locations YAML file
    location_file = rospy.get_param("location_file")
    print(f"Using location file: {location_file}")
    with open(location_file, "r") as f:
        locations = yaml.load(f, Loader=yaml.FullLoader)
    
    # Distance from navigation waypoint to spawn object
    block_spawn_offset = 0.5

    # Spawn blocks in random locations
    model_names = ["red_block", "green_block", "blue_block"]
    sampled_locs = random.sample(list(locations.keys()), len(model_names))
    for mdl, loc in zip(model_names, sampled_locs):
        x, y, theta = locations[loc]
        x += block_spawn_offset*math.cos(theta)
        y += block_spawn_offset*math.sin(theta)
        spawn_model(mdl, x, y, theta)
