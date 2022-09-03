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
import threading
import py_trees
from py_trees.common import OneShotPolicy
from ament_index_python.packages import get_package_share_directory

from tb3_behaviors.navigation import GoToPose, GetLocationFromQueue
from tb3_behaviors.vision import LookForObject


default_location_file = os.path.join(
    get_package_share_directory("tb3_worlds"),
    "maps", "sim_house_locations.yaml")

class TestNavigationNode(Node):
    def __init__(self):
        super().__init__("navigation_test_node")
        self.declare_parameter("location_file", value=default_location_file)
        self.declare_parameter("tree_type", value="naive")
        self.declare_parameter("enable_vision", value=True)
        self.declare_parameter("target_color", value="blue")

        # Parse locations YAML file and shuffle the location list.
        location_file = self.get_parameter("location_file").value
        print(f"Using location file: {location_file}")
        with open(location_file, "r") as f:
            self.locations = yaml.load(f, Loader=yaml.FullLoader)
        self.loc_list = list(self.locations.keys())
        random.shuffle(self.loc_list)

        # Get vision and target color parameters
        self.enable_vision = self.get_parameter("enable_vision").value
        self.target_color = self.get_parameter("target_color").value
        self.get_logger().info(f"Looking for color {self.target_color}...")

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
        if self.enable_vision:
            selector = py_trees.composites.Selector(name="navigation")
            for loc in self.loc_list:
                pose = self.locations[loc]
                selector.add_child(
                    py_trees.decorators.OneShot(
                        py_trees.composites.Sequence(
                            name=f"search_{loc}",
                            children=[
                                GoToPose(f"go_to_{loc}", pose, self),
                                LookForObject(f"find_{self.target_color}_{loc}",
                                              self.target_color, self)
                            ]
                        ),
                        policy=OneShotPolicy.ON_COMPLETION
                    )
                )
            root = py_trees.decorators.OneShot(selector)
        else:
            seq = py_trees.composites.Sequence(name="navigation")
            for loc in self.loc_list:
                pose = self.locations[loc]
                seq.add_child(GoToPose(f"go_to_{loc}", pose, self))
            root = py_trees.decorators.OneShot(seq)
        return py_trees.trees.BehaviourTree(root)

    def create_queue_tree(self):
        """ Create behavior tree by picking a next location from a queue """
        bb = py_trees.blackboard.Blackboard()
        bb.set("loc_list", self.loc_list)

        seq = py_trees.composites.Sequence(name="search")
        seq.add_children([
            GetLocationFromQueue("get_next_location", self.locations),
            GoToPose("go_to_location", None, self)
        ])
        if self.enable_vision:
            seq.add_child(LookForObject(f"find_{self.target_color}",
                                         self.target_color, self))
        root = py_trees.decorators.OneShot(seq)
        return py_trees.trees.BehaviourTree(root)

    def start_behavior_tree(self, tree, period_s=1.0, timeout_s=10.0):
        tree.setup(timeout=timeout_s)
        tree.add_post_tick_handler(self.display_tree)
        t = threading.Thread(target=tree.tick_tock, args=(period_s * 1000.0,))
        t.start()
        rclpy.spin(self)
        self.ros_tree.shutdown()
        rclpy.shutdown()

    def display_tree(self, tree):
        print("Current behavior:")
        print(py_trees.display.unicode_tree(tree.root))


if __name__=="__main__":
    rclpy.init()
    nav = TestNavigationNode()
    root = nav.create_behavior_tree()
    nav.start_behavior_tree(root, 1.0)
