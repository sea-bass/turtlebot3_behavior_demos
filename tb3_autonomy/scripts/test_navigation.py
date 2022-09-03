#!/usr/bin/env python3

"""
Navigation test node for the TurtleBot3.

This script relies on a YAML file of potential navigation locations, 
which is listed as a `location_file` ROS parameter.
"""

import os
import yaml
import random
import rclpy
from rclpy.node import Node
import threading
import py_trees
from ament_index_python.packages import get_package_share_directory
from tb3_behaviors.navigation import GoToPose, GetLocationFromQueue


default_location_file = os.path.join(
    get_package_share_directory("tb3_worlds"),
    "maps", "sim_house_locations.yaml")

class TestNavigationNode(Node):
    def __init__(self):
        super().__init__("navigation_test_node")
        self.declare_parameter("location_file", value=default_location_file)
        self.declare_parameter("tree_type", value="naive")

        # Parse locations YAML file and shuffle the location list.
        location_file = self.get_parameter("location_file").value
        print(f"Using location file: {location_file}")
        with open(location_file, "r") as f:
            self.locations = yaml.load(f, Loader=yaml.FullLoader)
        self.loc_list = list(self.locations.keys())
        random.shuffle(self.loc_list)

    def create_behavior_tree(self):
        tree_type = self.get_parameter("tree_type").value
        if tree_type == "naive":
            return self.create_naive_tree()
        elif tree_type == "queue":
            return self.create_queue_tree()
        else:
            self.get_logger().info(f"Invalid behavior tree type {tree_type}.")

    def create_naive_tree(self):
        """ Create behavior tree with explicit nodes for each location. """
        seq = py_trees.composites.Sequence(name="navigation")
        for loc in self.loc_list:
            pose = self.locations[loc]
            seq.add_child(GoToPose(f"go_to_{loc}", pose, self))
        root = py_trees.decorators.OneShot(seq)
        return py_trees.trees.BehaviourTree(root)

    def create_queue_tree(self):
        """ Create behavior tree by picking a next location from a queue """
        # Create behavior tree from shuffled locations
        bb = py_trees.blackboard.Blackboard()
        bb.set("loc_list", self.loc_list)

        root = py_trees.composites.Sequence(name="search")
        root.add_children([
            GetLocationFromQueue("get_next_location", self.locations),
            GoToPose("go_to_location", None, self)
        ])
        return py_trees.trees.BehaviourTree(root)

    def start_behavior_tree(self, tree, period_s=1.0, timeout_s=10.0):
        tree.setup(timeout=timeout_s)
        t = threading.Thread(target=tree.tick_tock, args=(period_s * 1000.0,))
        t.start()
        rclpy.spin(self)
        self.ros_tree.shutdown()
        rclpy.shutdown()


if __name__=="__main__":
    rclpy.init()
    nav = TestNavigationNode()
    root = nav.create_behavior_tree()
    nav.start_behavior_tree(root, 1.0)
