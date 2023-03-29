#!/usr/bin/env python
# -*- coding: utf-8 -*-

# PyTrees imports
import py_trees
import py_trees_ros.actions
import py_trees.console as console\

# ROS imports
import rospy

# Tiago imports
from tiago_dual_pick_place.msg import PlaceAutoObjectAction, PlaceObjectGoal

# AGI imports
from utils.robot_utils import get_gripper_status, get_arm
from perception_interface import StackedCubesPub

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
        self.stacked_cubes_pub = StackedCubesPub()
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
        rospy.logdebug("Initialising PlaceBehaviour")
        super(PlaceBehaviour, self).initialise()

    def update(self):
        action_result = super(PlaceBehaviour, self).update()
        if action_result == py_trees.common.Status.RUNNING:
            return py_trees.common.Status.RUNNING
        elif action_result == py_trees.common.Status.SUCCESS:
            # Update the pose of the cube to the target location
            # And mark it as in the stack so it is not picked up again
            placed_cube = self._blackboard.get("next_cube")
            target_location = self._blackboard.get("target_location")
            self._blackboard.get("cubes_in_stack").append(placed_cube)
            if self._blackboard.get("cubes_not_in_center") != None:
                self._blackboard.get("cubes_not_in_center").append(placed_cube)
            placed_cube._properties["in_stack"] = True
            placed_cube._properties["fixed"] = True
            # inflate to block for collision avoidance
            placed_cube.resize_block((0.055, 0.055, 0.045))
            target_location.pose.position.z -= 0.015  # offset correction!
            # target_location.pose.orientation.x = 0
            # target_location.pose.orientation.y = 0
            # target_location.pose.orientation.z = 0
            # target_location.pose.orientation.w = 1
            placed_cube._pose = target_location
            placed_cube.update()

            # interface to perception 
            # target_location: x, y relevant for ignoring the stacked cubes in the perception
            # cubes in stack to adjust number of cubes for remaining cubes
            n_stacked_cubes = len(self._blackboard.get("cubes_in_stack"))
            
            # TODO: uncomment to test!
            # print(target_location)
            # print(stacked_cubes)
            # publishing it with an extra publicher class and msg
            # self.stacked_cubes_pub(location, n_stacked_cubes)

            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

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
