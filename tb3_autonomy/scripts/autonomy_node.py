#!/usr/bin/env python3

"""
Autonomy node for the TurtleBot3.

This script relies on a YAML file of potential navigation locations, 
which is listed as a `location_file` ROS parameter.

Example usage:
  ros2 run tb3_autonomy autonomy_node.py
  ros2 run tb3_autonomy autonomy_node.py --ros-args -p location_file:=/path/to/my/file.yaml
  ros2 run tb3_autonomy autonomy_node.py --ros-args -p tree_type:=queue -p target_color:=green
"""

import os
import yaml
import random
import rclpy
from rclpy.node import Node
import time
import py_trees
import py_trees_ros
from py_trees.common import OneShotPolicy
from ament_index_python.packages import get_package_share_directory

from tb3_behaviors.navigation import GoToPose, GetLocationFromQueue
from tb3_behaviors.vision import LookForObject


default_location_file = os.path.join(
    get_package_share_directory("tb3_worlds"),
    "maps", "sim_house_locations.yaml")

class AutonomyBehavior:
    def __init__(self, tree_type="queue", target_color="blue", enable_vision=True):
        # Parse locations YAML file and shuffle the location list.
        # location_file = self.get_parameter("location_file").value
        location_file = default_location_file
        with open(location_file, "r") as f:
            self.locations = yaml.load(f, Loader=yaml.FullLoader)
        self.loc_list = list(self.locations.keys())
        random.shuffle(self.loc_list)

        # Create and setup the behavior tree
        self.tree_type = tree_type
        self.enable_vision = enable_vision
        self.target_color = target_color
        self.create_behavior_tree(self.tree_type)
        self.tree.node.get_logger().info(
            f"Using location file: {location_file}")

        # Set up parameters
        #self.declare_parameter("location_file", value=default_location_file)
        #self.declare_parameter("tree_type", value="naive")
        #self.declare_parameter("enable_vision", value=True)
        #self.declare_parameter("target_color", value="blue")
        #self.enable_vision = self.get_parameter("enable_vision").value
        #self.target_color = self.get_parameter("target_color").value
        self.tree.node.get_logger().info(
            f"Looking for color {self.target_color}...")

    def create_behavior_tree(self, tree_type):
        if tree_type == "naive":
            self.tree = self.create_naive_tree()
        elif tree_type == "queue":
            self.tree = self.create_queue_tree()
        else:
            self.get_logger().info(f"Invalid behavior tree type {tree_type}.")

    def create_naive_tree(self):
        """ Create behavior tree with explicit nodes for each location. """
        if self.enable_vision:
            selector = py_trees.composites.Selector(name="navigation")
            root = py_trees.decorators.OneShot(selector)
            tree = py_trees_ros.trees.BehaviourTree(root, unicode_tree_debug=True)
            tree.setup(timeout=15.0)  # Need the tree node to exist

            for loc in self.loc_list:
                pose = self.locations[loc]
                selector.add_child(
                    py_trees.decorators.OneShot(
                        py_trees.composites.Sequence(
                            name=f"search_{loc}",
                            children=[
                                GoToPose(f"go_to_{loc}", pose, tree.node),
                                LookForObject(f"find_{self.target_color}_{loc}",
                                              self.target_color, tree.node)
                            ]
                        ),
                        policy=OneShotPolicy.ON_COMPLETION
                    )
                )
            
        else:
            seq = py_trees.composites.Sequence(name="navigation")
            root = py_trees.decorators.OneShot(seq)
            tree = py_trees_ros.trees.BehaviourTree(root, unicode_tree_debug=True)
            tree.setup(timeout=15.0)  # Need the tree node to exist

            for loc in self.loc_list:
                pose = self.locations[loc]
                seq.add_child(GoToPose(f"go_to_{loc}", pose, self))

        return tree

    def create_queue_tree(self):
        """ Create behavior tree by picking a next location from a queue """
        bb = py_trees.blackboard.Blackboard()
        bb.set("loc_list", self.loc_list)

        seq = py_trees.composites.Sequence(name="search")
        root = py_trees.decorators.OneShot(seq)
        tree = py_trees_ros.trees.BehaviourTree(root, unicode_tree_debug=True)
        tree.setup(timeout=15.0)  # Need the tree node to exist

        seq.add_children([
            GetLocationFromQueue("get_next_location", self.locations),
            GoToPose("go_to_location", None, tree.node)
        ])
        if self.enable_vision:
            seq.add_child(LookForObject(f"find_{self.target_color}",
                                        self.target_color, tree.node))
        return tree

    def execute(self, period=1.0):
        self.tree.tick_tock(period_ms=period*1000.0)
        rclpy.spin(self.tree.node)
        rclpy.shutdown()


if __name__=="__main__":
    rclpy.init()
    behavior = AutonomyBehavior(
        tree_type="queue",
        target_color="blue",
        enable_vision=True)
    behavior.execute()
