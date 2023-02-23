#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Time
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import JointTolerance
from control_msgs.srv import QueryTrajectoryState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


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
        rospy.logerr("Service call failed: " + e)

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
    Args:
        left_right (str): The side of the gripper to open
    Returns:
        bool: True if successful, False if not

    Example:
        >>> open_gripper("left")
    """
    if left_right == "left":
        topic_name = "/parallel_gripper_left_controller/follow_joint_trajectory/goal"
        gripper_joint_names = [
            "gripper_left_left_finger_joint", "gripper_left_right_finger_joint"
        ]

    elif left_right == "right":
        topic_name = "/parallel_gripper_right_controller/follow_joint_trajectory/goal"
        gripper_joint_names = [
            "gripper_right_left_finger_joint",
            "gripper_right_right_finger_joint"
        ]

    pub = rospy.Publisher(topic_name, FollowJointTrajectoryGoal, queue_size=10)
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = [
        gripper_joint_names[0], gripper_joint_names[1]
    ]
    goal.trajectory.points = [
        JointTrajectoryPoint(positions=[0.048, 0.048],
                             velocities=[0.0, 0.0],
                             time_from_start=rospy.Duration(1.0))
    ]
    goal.path_tolerance = [
        JointTolerance(name=gripper_joint_names[0], position=0.001),
        JointTolerance(name=gripper_joint_names[1], position=0.001)
    ]
    goal.goal_tolerance = [
        JointTolerance(name=gripper_joint_names[0], position=0.001),
        JointTolerance(name=gripper_joint_names[1], position=0.001)
    ]
    goal.goal_time_tolerance = rospy.Duration(0.0)

    pub.publish(goal)

    return True
