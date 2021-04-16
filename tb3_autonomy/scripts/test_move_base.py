#!/usr/bin/env python3

"""
Script that tests the `move_base` action client.

Example usage:
  Python: python3 test_move_base.py --x 0 --y 2 --theta 1.57
  rosrun: rosrun tb3_autonomy test_move_base.py --x 0 --y 2 --theta 1.57
"""

import tf
import rospy
import argparse
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

if __name__=="__main__":
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Move base test script")
    parser.add_argument("--x", type=str, default="1.0")
    parser.add_argument("--y", type=str, default="0.0")
    parser.add_argument("--theta", type=str, default="0.0")
    args = parser.parse_args()

    # Start ROS node and action client
    rospy.init_node("test_move_base")
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()

    # Package up a goal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = float(args.x)
    goal.target_pose.pose.position.y = float(args.y)
    quat = tf.transformations.quaternion_from_euler(0, 0, float(args.theta))
    goal.target_pose.pose.orientation.x = quat[0]
    goal.target_pose.pose.orientation.y = quat[1]
    goal.target_pose.pose.orientation.z = quat[2]
    goal.target_pose.pose.orientation.w = quat[3]

    # Send goal to the move_base action server
    print(f"Going to [x: {args.x}, y: {args.y}, theta: {args.theta}]...")
    client.send_goal(goal)
    result = client.wait_for_result()
    print(f"Action complete with result: {result}")
