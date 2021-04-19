#!/usr/bin/env python3

"""
Autonomy script for the TurtleBot3

Combines simple vision and navigation behaviors
"""

import tf
import yaml
import rospy
import random
import py_trees
import py_trees_ros
from tb3_behaviors.navigation import GoToPose
from tb3_behaviors.vision import LookForObject
from tb3_behaviors.decorators import OneShot


if __name__=="__main__":
    # Start ROS node
    rospy.init_node("autonomy_node")

    # Get target color (red, green, or blue)
    target_color = rospy.get_param("target_color")
    print(f"Targeting object color: {target_color}")

    # Parse locations YAML file
    location_file = rospy.get_param("location_file")
    print(f"Using location file: {location_file}")
    with open(location_file, "r") as f:
        locations = yaml.load(f, Loader=yaml.FullLoader)

    # Create behavior tree from shuffled locations and object name
    root = py_trees.composites.Selector(name="root")
    loc_list = list(locations.keys())
    random.shuffle(loc_list)
    for loc in loc_list:
        target_pose = locations[loc]
        root.add_child(
            OneShot(
                py_trees.composites.Sequence(
                    name=f"search_{loc}", 
                    children=[GoToPose(f"go_to_{loc}", target_pose), 
                            LookForObject(f"find_{target_color}_{loc}", target_color)]
                ),
                name = f"one_shot_{loc}",
                on_successful_completion = False
            )
      )
    tree = py_trees.trees.BehaviourTree(root)
    ros_tree = py_trees_ros.trees.BehaviourTree(root)
    ros_tree.setup(timeout=10.0)
    py_trees.logging.level = py_trees.logging.Level.INFO

    # Tick the tree until a terminal state is reached
    done = False
    while not rospy.is_shutdown() and not done:
        ros_tree.tick()
        if ros_tree.root.status == py_trees.common.Status.SUCCESS:
            print("Behavior tree succeeded")
            done = True
        elif ros_tree.root.status == py_trees.common.Status.FAILURE:
            print("Behavior tree failed.")
            done = True
        rospy.sleep(0.5)
    rospy.spin()