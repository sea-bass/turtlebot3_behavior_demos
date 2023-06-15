"""
Navigation behaviors for TurtleBot3.
"""

import py_trees
import transforms3d

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


class GetLocationFromQueue(py_trees.behaviour.Behaviour):
    """Gets a location name from the queue"""

    def __init__(self, name, location_dict):
        super(GetLocationFromQueue, self).__init__(name)
        self.location_dict = location_dict
        self.bb = py_trees.blackboard.Blackboard()

    def update(self):
        """Checks for the status of the navigation action"""
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
    """Wrapper behavior around the `move_base` action client"""

    def __init__(self, name, pose, node):
        super(GoToPose, self).__init__(name)
        self.pose = pose
        self.client = None
        self.node = node
        self.bb = py_trees.blackboard.Blackboard()

    def initialise(self):
        """Sends the initial navigation action goal"""
        # Check if there is a pose available in the blackboard
        try:
            target_pose = self.bb.get("target_pose")
            if target_pose is not None:
                self.pose = target_pose
        except:
            pass

        self.client = ActionClient(self.node, NavigateToPose, "/navigate_to_pose")
        self.client.wait_for_server()

        self.goal_status = None
        x, y, theta = self.pose
        self.logger.info(f"Going to [x: {x}, y: {y}, theta: {theta}] ...")
        self.goal = self.create_move_base_goal(x, y, theta)
        self.send_goal_future = self.client.send_goal_async(self.goal)
        self.send_goal_future.add_done_callback(self.goal_callback)

    def goal_callback(self, future):
        res = future.result()
        if res is None or not res.accepted:
            return
        future = res.get_result_async()
        future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        # If there is a result, we consider navigation completed and save the
        # result code to be checked in the `update()` method.
        self.goal_status = future.result().status

    def update(self):
        """Checks for the status of the navigation action"""
        # If there is a result, we can check the status of the action directly.
        # Otherwise, the action is still running.
        if self.goal_status is not None:
            if self.goal_status == GoalStatus.STATUS_SUCCEEDED:
                return py_trees.common.Status.SUCCESS
            else:
                return py_trees.common.Status.FAILURE
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.info(f"Terminated with status {new_status}")
        self.client = None
        self.bb.set("target_pose", None)

    def create_move_base_goal(self, x, y, theta):
        """Creates a MoveBaseGoal message from a 2D navigation pose"""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        quat = transforms3d.euler.euler2quat(0, 0, theta)
        goal.pose.pose.orientation.w = quat[0]
        goal.pose.pose.orientation.x = quat[1]
        goal.pose.pose.orientation.y = quat[2]
        goal.pose.pose.orientation.z = quat[3]
        return goal
