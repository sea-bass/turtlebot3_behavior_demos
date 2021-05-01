#!/usr/bin/env python3

"""
Navigation test node for the TurtleBot3

This script relies on a YAML file of potential navigation locations, 
which is listed as a `/location_file` ROS parameter
"""

import tf
import yaml
import rospy
import random
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import py_trees
import py_trees_ros
from tb3_behaviors.navigation import GoToPose


if __name__=="__main__":
    # Start ROS node
    rospy.init_node("navigation_node")

    # Parse locations YAML file
    location_file = rospy.get_param("location_file")
    print(f"Using location file: {location_file}")
    with open(location_file, "r") as f:
        locations = yaml.load(f, Loader=yaml.FullLoader)

    # Navigate goals randomly
    use_behavior_trees = True
    if use_behavior_trees:
        # Create behavior tree from shuffled locations
        root = py_trees.composites.Sequence(name="navigation")
        loc_list = list(locations.keys())
        random.shuffle(loc_list)
        for loc in loc_list:
            pose = locations[loc]
            root.add_child(GoToPose(f"go_to_{loc}", pose))
            tree = py_trees.trees.BehaviourTree(root)
        ros_tree = py_trees_ros.trees.BehaviourTree(root)
        ros_tree.setup(timeout=10.0)
        py_trees.logging.level = py_trees.logging.Level.INFO

        # Tick the tree until a terminal state is reached
        while not rospy.is_shutdown():
            ros_tree.tick()
            if ros_tree.root.status == py_trees.common.Status.SUCCESS:
                print("Behavior tree succeeded")
                exit()
            elif ros_tree.root.status == py_trees.common.Status.FAILURE:
                print("Behavior tree failed.")
                exit()
            rospy.sleep(0.5)
      
    else:
        # Start move_base client and loop indefinitely
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        client.wait_for_server()
        while not rospy.is_shutdown():
            next_loc = random.choice(list(locations.keys()))
            x, y, theta = locations[next_loc]
            print(f"Selected {next_loc}")
            print(f"Going to [x: {x}, y: {y}, theta: {theta}] ...")
            goal = create_move_base_goal(x, y, theta)
            client.send_goal(goal)
            result = client.wait_for_result()
            print(f"Action complete with result: {result}")
