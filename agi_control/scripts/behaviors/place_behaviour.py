#!/usr/bin/env python
# -*- coding: utf-8 -*-
import pdb
import rospy
import py_trees
import py_trees_ros.actions
import py_trees.console as console

# tiago_dual_pick_place
from tiago_dual_pick_place.msg import PlaceAutoObjectAction, PlaceObjectGoal
from utils.robot_utils import get_gripper_status, get_arm


class PlaceBehaviour(py_trees_ros.actions.ActionClient):
    """Action Client for placing a block

    This class is a wrapper around the ActionClient class from py_trees_ros.
    It generates the goal for the action client and sets the action spec.
    """
    maps = None

    def __init__(self, name="Place Block"):
        """Constructor for PlaceBehaviour

        Args:
            name (str, optional): Name of the behavior. Defaults to "Place Block
        """
        super(PlaceBehaviour, self).__init__(name=name,
                                             action_spec=PlaceAutoObjectAction,
                                             action_namespace="/place_object")

    def setup(self, timeout):
        """Setup for the behavior

        Args:
            timeout (float): Timeout for the behavior

        Returns:
            bool: True if successful
        """
        self._blackboard = py_trees.blackboard.Blackboard()
        return super(PlaceBehaviour, self).setup(timeout)

    def initialise(self):
        self.action_goal = self.get_place_goal()
        pdb.set_trace()
        rospy.logdebug("Initialising PlaceBehaviour")
        super(PlaceBehaviour, self).initialise()

    def get_place_goal(self):
        """Generates the goal for the action client

        Returns:
            PlaceObjectGoal: The goal for the action client
        """
        console.loginfo("Getting place goal")
        place_goal = PlaceObjectGoal()

        place_goal.left_right = self._blackboard.get("next_cube").left_right
        console.loginfo("arm:{}".format(place_goal.left_right))

        next_cube = self._blackboard.get("next_cube")
        if next_cube is None:
            place_goal.object_name = ''
        else:
            place_goal.object_name = next_cube.id
            place_goal.target_pose = self._blackboard.get("target_location")
            console.loginfo("Place goal: {}".format(place_goal))
        console.logdebug("Place goal")
        return place_goal
