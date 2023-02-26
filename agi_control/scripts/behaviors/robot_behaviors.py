#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import pdb

# py_trees
import py_trees
import py_trees.console as console
import py_trees_ros.trees

# tiago_dual_pick_place
from tiago_dual_pick_place.msg import PlaceAutoObjectAction
from tiago_dual_pick_place.msg import PlaceAutoObjectGoal
from tiago_dual_pick_place.msg import PickUpObjectAction
from tiago_dual_pick_place.msg import PickUpObjectGoal


class PlaceBlock(py_trees_ros.actions.ActionClient):
    """Action Client for placing a block
    
    This class is a wrapper around the ActionClient class from py_trees_ros.
    It generates the goal for the action client and sets the action spec.
    
    Args:
        py_trees_ros.actions.ActionClient ([type]): [description]

    """

    def __init__(self, name="Place Block"):
        super(PlaceBlock, self).__init__(name=name,
                                         action_spec=PlaceAutoObjectAction,
                                         action_namespace="/place_object")

    def initialise(self):
        self.action_goal = self.get_place_goal()
        super(PlaceBlock, self).initialise()

    def get_place_goal(self):
        """Generates the goal for the action client
        
        Returns:
            PlaceAutoObjectGoal: The goal for the action client
        """
        console.loginfo("Getting place goal")
        blackboard = py_trees.blackboard.Blackboard()
        if blackboard.get("next_cube") is None:
            console.logwarn("No more blocks to place")
            return py_trees.common.Status.FAILURE

        place_goal = PlaceAutoObjectGoal()
        place_goal.target_pose = blackboard.get("next_stack_location")
        place_goal.object_name = blackboard.get("next_cube").id
        console.loginfo("Place goal: {}".format(place_goal))
        return place_goal

    def update(self):
        print("Updating place goal")
        blackboard = py_trees.blackboard.Blackboard()
        if blackboard.get("next_cube") is None:
            console.loginfo("No more blocks to place")
            return py_trees.common.Status.FAILURE
        ret = super(PlaceBlock, self).update()
        # If the action succeeded, add the block to the stack and clear the next block variable
        if ret == py_trees.common.Status.SUCCESS:
            blackboard.cubes_in_stack.append(blackboard.next_cube)
            blackboard.set("next_cube", None)
        return ret