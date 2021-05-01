"""
Navigation behaviors for TurtleBot3
"""

import tf
import rospy
import py_trees
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class GetLocationFromQueue(py_trees.behaviour.Behaviour):
    """ Gets a location name from the queue """
    def __init__(self, name, location_dict):
        super(GetLocationFromQueue, self).__init__(name)
        self.location_dict = location_dict
        self.bb = py_trees.blackboard.Blackboard()

    def update(self):
        """ Checks for the status of the navigation action """
        loc_list = self.bb.get("loc_list")
        if len(loc_list) == 0:
            self.logger.info("No locations available")
            return py_trees.common.Status.FAILURE
        else:
            target_location = loc_list.pop()
            self.logger.info(f"Selected location {target_location}")
            target_pose = self.location_dict[target_location]
            self.bb.set("target_pose", target_pose)
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.logger.info(f"Terminated with status {new_status}")


class GoToPose(py_trees.behaviour.Behaviour):
    """ Wrapper behavior around the `move_base` action client """

    def __init__(self, name, pose=None):
        super(GoToPose, self).__init__(name)
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        self.pose = pose
        self.bb = py_trees.blackboard.Blackboard()

    def initialise(self):
        """ Sends the initial navigation action goal """
        # Check if there is a pose available in the blackboard
        target_pose = self.bb.get("target_pose")
        if target_pose is not None:
            self.pose = target_pose
        
        x, y, theta = self.pose
        self.logger.info(f"Going to [x: {x}, y: {y}, theta: {theta}] ...")
        goal = create_move_base_goal(x, y, theta)
        self.client.send_goal(goal)
        rospy.sleep(0.5)    # Ensure goal was received before checking state

    def update(self):
        """ Checks for the status of the navigation action """
        status = self.client.get_state()
        if status == GoalStatus.SUCCEEDED:
            return py_trees.common.Status.SUCCESS
        if status == GoalStatus.ACTIVE:
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.info(f"Terminated with status {new_status}")
        self.bb.set("target_pose", None)


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
