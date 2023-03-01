#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from std_msgs.msg import Time
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import JointTolerance
from control_msgs.srv import QueryTrajectoryState
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import numpy as np
from arm_selection import Arm, load_maps

#---------------------------------#
# Gripper Stuff
#---------------------------------#
def get_gripper_status(left_right):
    """This function takes 'left' or 'right' as input and checks if the gripper is open or closed.
    Args:
        left_right (str): The side of the gripper to check
    Returns:
        str: 'open', 'closed', 'with_block' or 'unknown'

    Example:
        >>> get_gripper_status("left")
    """
    gripper_pos = []
    gripper_vel = []

    if left_right == "left":
        srv_name = "/gripper_left_controller/query_state"
    elif left_right == "right":
        srv_name = "/gripper_right_controller/query_state"

    rospy.wait_for_service(srv_name)
    try:
        srv = rospy.ServiceProxy(srv_name, QueryTrajectoryState)
        resp = srv(rospy.Time.now())
        gripper_pos = resp.position
        gripper_vel = resp.velocity
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed")
        return 'error'

    if gripper_pos[0] > 0.035:
        return 'open'
    elif gripper_pos[0] < 0.035 and gripper_pos[0] > 0.02:
        return 'with_block'
    elif gripper_pos[0] < 0.02:
        return 'closed'
    else:
        return 'unknown'


def open_gripper(left_right):
    """This function takes 'left' or 'right' as input and opens the gripper.
    It calls the play_motion action server to execute the open_left/right action.
    Args:
        left_right (str): The side of the gripper to open
    Returns:
        bool: True if the gripper is open, False otherwise

    Example:
        >>> open_gripper("left")
    """
    if left_right == "left":
        play_motion_name = "open_left"
    elif left_right == "right":
        play_motion_name = "open_right"

    client = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
    client.wait_for_server()

    goal = PlayMotionGoal()
    goal.motion_name = play_motion_name
    goal.skip_planning = True
    goal.priority = 0

    client.send_goal(goal)
    client.wait_for_result()

    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
        return True
    else:
        return False
    
def get_arm(map_list,next_cube):
    pose = np.array(([[next_cube.pose.pose.position.x,next_cube.pose.pose.position.y,next_cube.pose.pose.position.z,\
                    next_cube.pose.pose.orientation.x,next_cube.pose.pose.orientation.y,next_cube.pose.pose.orientation.z]]))
    arm = Arm(pose, map_list[0], map_list[0])
    return arm.getArm()

    
