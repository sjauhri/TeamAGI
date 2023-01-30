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

from scene_behaviors import GetNextStackLocation


class PickBlock(py_trees_ros.actions.ActionClient):
    """Action Client for picking up a block
    
    This class is a wrapper around the ActionClient class from py_trees_ros.
    It generates the goal for the action client and sets the action spec.
    """

    def __init__(self, name="Pick Block"):
        """Constructor for PickBlock
        
        Args:
            name (str, optional): Name of the behavior. Defaults to "Pick Block

        """
        super(PickBlock, self).__init__(name=name,
                                        action_spec=PickUpObjectAction,
                                        action_namespace="/pickup_object")

    def initialise(self):
        self.action_goal = self.get_pick_up_goal()
        super(PickBlock, self).initialise()

    def get_pick_up_goal(self):
        """Generates the goal for the action client
        
        Returns:
            PickUpObjectGoal: The goal for the action client
        """
        console.loginfo("Getting pick up goal")
        blackboard = py_trees.blackboard.Blackboard()
        pick_up_goal = PickUpObjectGoal()
        # pick_up_goal.left_right = 'left'
        
        # select arm
        getNextStackLocation = GetNextStackLocation()
        try:
            pick_up_goal.left_right = getNextStackLocation.left_right
        except:
            pick_up_goal.left_right = "left" 
            
        next_block = blackboard.get("next_block")
        if next_block is None:
            pick_up_goal.object_name = ''
        else:
            pick_up_goal.object_name = next_block.id
        console.logdebug("Pick up goal: {}".format(pick_up_goal))
        return pick_up_goal


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
        if blackboard.get("next_block") is None:
            console.logwarn("No more blocks to place")
            return py_trees.common.Status.FAILURE

        place_goal = PlaceAutoObjectGoal()
        place_goal.target_pose = blackboard.get("next_stack_location")
        place_goal.object_name = blackboard.get("next_block").id
        console.loginfo("Place goal: {}".format(place_goal))
        return place_goal

    def update(self):
        print("Updating place goal")
        blackboard = py_trees.blackboard.Blackboard()
        if blackboard.get("next_block") is None:
            console.loginfo("No more blocks to place")
            return py_trees.common.Status.FAILURE
        ret = super(PlaceBlock, self).update()
        # If the action succeeded, add the block to the stack and clear the next block variable
        if ret == py_trees.common.Status.SUCCESS:
            blackboard.blocks_in_stack.append(blackboard.next_block)
            blackboard.next_block = ''
        return ret