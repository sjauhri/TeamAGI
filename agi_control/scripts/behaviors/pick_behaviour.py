#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

# py_trees
import py_trees
import py_trees.console as console
import py_trees_ros.trees

# tiago_dual_pick_place
from tiago_dual_pick_place.msg import PickUpObjectAction
from tiago_dual_pick_place.msg import PickUpObjectGoal

# arm selection
import numpy as np
from arm_selection import Arm, load_maps
from utils.robot_utils import get_gripper_status


class PickBehaviour(py_trees_ros.actions.ActionClient):
    """Action Client for picking up a block
    
    This class is a wrapper around the ActionClient class from py_trees_ros.
    It generates the goal for the action client and sets the action spec.
    """
    maps = None

    def __init__(self, name="Pick Block"):
        """Constructor for PickBlock
        
        Args:
            name (str, optional): Name of the behavior. Defaults to "Pick Block

        """
        super(PickBehaviour, self).__init__(name=name,
                                            action_spec=PickUpObjectAction,
                                            action_namespace="/pickup_object")

    def setup(self, timeout):
        """Setup for the behavior
        
        Args:
            timeout (float): Timeout for the behavior
        
        Returns:
            bool: True if successful
        """
        # the reach map is only initialezed once
        if PickBehaviour.maps is None:
            PickBehaviour.maps = load_maps()
        self.map_list = PickBehaviour.maps

        self._blackboard = py_trees.blackboard.Blackboard()
        return super(PickBehaviour, self).setup(timeout)

    def initialise(self):
        self.action_goal = self.get_pick_up_goal()
        gripper_status = get_gripper_status('left')
        self._blackboard.set("gripper_status", gripper_status)
        rospy.logdebug("Initialising PickBehaviour")
        super(PickBehaviour, self).initialise()

    def select_arm(self):
        # input pose -> selected arm
        # pose(1,6) -> single string, pose(n,6),n>=2 -> list of strings
        next_cube = self._blackboard.get("next_cube")
        pose = np.array(([[next_cube.pose.pose.position.x,next_cube.pose.pose.position.y,next_cube.pose.pose.position.z,\
                        next_cube.pose.pose.orientation.x,next_cube.pose.pose.orientation.y,next_cube.pose.pose.orientation.z]]))
        arm = Arm(pose, self.map_list[0], self.map_list[0])
        return arm.getArm()

    def get_pick_up_goal(self):
        """Generates the goal for the action client
        
        Returns:
            PickUpObjectGoal: The goal for the action client
        """
        console.loginfo("Getting pick up goal")
        pick_up_goal = PickUpObjectGoal()

        # test: arm selection
        # pick_up_goal.left_right = 'left'
        pick_up_goal.left_right = self.select_arm()
        console.loginfo("arm:{}".format(pick_up_goal.left_right))

        next_cube = self._blackboard.get("next_cube")
        if next_cube is None:
            pick_up_goal.object_name = ''
        else:
            pick_up_goal.object_name = next_cube.id
        console.logdebug("Pick up goal")
        return pick_up_goal
