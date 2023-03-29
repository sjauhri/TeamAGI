#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import pdb
import actionlib
from std_msgs.msg import Time
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import JointTolerance
from control_msgs.msg import JointTrajectoryControllerState
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
import numpy as np
from behaviors.arm_selection import Arm


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
        topic_name = "/parallel_gripper_left_controller/state"
    elif left_right == "right":
        topic_name = "/parallel_gripper_right_controller/state"
    else:
        print("Cant Check GRIPPER STATUS (in robot_utils.py)!")


    try:
        rospy.wait_for_message(topic_name,
                               JointTrajectoryControllerState,
                               timeout=1.0)
        msg = rospy.wait_for_message(topic_name,
                                     JointTrajectoryControllerState,
                                     timeout=1.0)
        gripper_pos_actual = msg.actual.positions
        gripper_pos_desired = msg.desired.positions
        gripper_pos_error = msg.error.positions

    except rospy.ROSException:
        return "unknown"

    if gripper_pos_actual[0] > 0.07:
        return "open"
    # elif gripper_pos_error[0] < -0.0001:
    elif gripper_pos_actual[0] > 0.040:
        return "with_cube"
    elif gripper_pos_actual[0] < 0.040:
        return "closed"
    else:
        return "unknown"


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


def get_arm(cube):
    pose = np.array(([[cube.pose.pose.position.x,cube.pose.pose.position.y,cube.pose.pose.position.z,\
                    cube.pose.pose.orientation.x,cube.pose.pose.orientation.y,cube.pose.pose.orientation.z, cube.pose.pose.orientation.w]]))
    arm = Arm(pose)
    arm = arm.getArm()
    if arm is None:
        arm = 'left'
    return arm
