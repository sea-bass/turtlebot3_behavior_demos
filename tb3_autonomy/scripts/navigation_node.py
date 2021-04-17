#!/usr/bin/env python3

"""
Navigation script for the TurtleBot3

This script relies on a YAML file of potential navigation locations, 
which is listed as a `/location_file` ROS parameter
"""

import tf
import yaml
import rospy
import random
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def create_move_base_goal(x, y, theta):
    """ Creates a MoveBaseGoal message from a 2D navigation pose """
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    quat = tf.transformations.quaternion_from_euler(0, 0, theta)
    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]
    return goal

if __name__=="__main__":
    # Start ROS node and action client
    rospy.init_node("test_move_base")
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()

    # Parse locations YAML file
    location_file = rospy.get_param("location_file")
    print(f"Using location file: {location_file}")
    with open(location_file, "r") as f:
        locations = yaml.load(f, Loader=yaml.FullLoader)

    # Send goals randomly
    while not rospy.is_shutdown():
      next_loc = random.choice(list(locations.keys()))
      x, y, theta = locations[next_loc]
      print(f"Selected {next_loc}")
      print(f"Going to [x: {x}, y: {y}, theta: {theta}] ...")
      goal = create_move_base_goal(x, y, theta)
      client.send_goal(goal)
      result = client.wait_for_result()
      print(f"Action complete with result: {result}")
    