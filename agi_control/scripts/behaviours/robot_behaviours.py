#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import pdb

# py_trees
import py_trees
import py_trees.console as console
import py_trees_ros.trees

# actionlib
from actionlib import SimpleActionClient

# play_motion_msgs
from play_motion_msgs.msg import PlayMotionAction
from play_motion_msgs.msg import PlayMotionGoal

# tiago_dual_pick_place
from tiago_dual_pick_place.msg import PlaceAutoObjectAction
from tiago_dual_pick_place.msg import PlaceAutoObjectGoal
from tiago_dual_pick_place.msg import PickUpObjectAction
from tiago_dual_pick_place.msg import PickUpObjectGoal

from reachability import Arm


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
        next_block = blackboard.get("next_block")
        if next_block is None:
            pick_up_goal.object_name = ''
            console.loginfo("No next block, using empty string")
        else:
            pick_up_goal.object_name = next_block.id
            pick_up_goal.left_right = blackboard.get("left_right")
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
        return place_goal

    def update(self):
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


class SelectArm(py_trees.behaviour.Behaviour):
    """Selects the arm to use for picking up a block
    
    This behavior is used to select the arm to use for picking up a block.
    It uses the reachability module to determine the arm to use.
    """

    def __init__(self, name="Select Arm"):
        """Constructor for SelectArm
        
        Args:
            name (str, optional): Name of the behavior. Defaults to "Select Arm"
        """
        super(SelectArm, self).__init__(name=name)

    def update(self):
        """Selects the arm to use for picking up a block
        
        Returns:
            py_trees.common.Status: SUCCESS if the arm was selected, FAILURE otherwise
        """
        blackboard = py_trees.blackboard.Blackboard()
        if blackboard.get("next_block") is None:
            console.loginfo("No more blocks to pick up")
            return py_trees.common.Status.FAILURE
        try:
            arm = Arm(
                blackboard.get("next_block").primitive_poses[0].position.x,
                blackboard.get("next_block").primitive_poses[0].position.y,
                blackboard.get("next_block").primitive_poses[0].position.z)

            left_right = arm.getArm()
        except:
            left_right = "left"
            console.loginfo("Could not determine arm, using left")

        if blackboard.get("left_right") is None:
            blackboard.set("left_right_old", left_right)
        else:
            blackboard.set("left_right_old", blackboard.get("left_right"))

        blackboard.set("left_right", left_right)
        console.loginfo("Selected arm: {}".format(left_right))
        return py_trees.common.Status.SUCCESS


class MoveArmToRest(py_trees_ros.actions.ActionClient):
    """Action Client for moving the arm to the rest position
    
    This class is a wrapper around the ActionClient class from py_trees_ros.
    It generates the goal for the action client and sets the action spec.
    """

    def __init__(self, name="Move Arm To Rest"):
        """Constructor for MoveArmToRest
        
        Args:
            name (str, optional): Name of the behavior. Defaults to "Move Arm To Rest"
        """

        super(MoveArmToRest, self).__init__(name=name,
                                            action_spec=PlayMotionAction,
                                            action_namespace="/play_motion")

    def initialise(self):
        self.left_right = py_trees.blackboard.Blackboard().get(
            "left_right_old")
        self.action_goal = self.get_move_arm_to_rest_goal()
        super(MoveArmToRest, self).initialise()

    def get_move_arm_to_rest_goal(self):
        """Generates the goal for the action client
        
        Returns:
            MoveArmToRestGoal: The goal for the action client
        """
        console.loginfo("Getting move arm to rest goal")
        move_arm_to_rest_goal = PlayMotionGoal()
        move_arm_to_rest_goal.motion_name = 'pregrasp_' + self.left_right[0]
        move_arm_to_rest_goal.skip_planning = False
        console.logdebug(
            "Move arm to rest goal: {}".format(move_arm_to_rest_goal))
        return move_arm_to_rest_goal
