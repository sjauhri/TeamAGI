#! /usr/bin/env python
# -*- coding: utf-8 -*-

# System imports
import pdb

# PyTrees imports
import py_trees
import py_trees.console as console

# AGI imports
from utils.robot_utils import open_gripper

# MoveIt imports
from moveit_commander import PlanningSceneInterface


class PlaceRecovery(py_trees.behaviour.Behaviour):
    """This class can be used to create a recovery behaviour for the place subtask.

    Example:
        >>> recovery = PlaceRecovery("Recovery")
    """

    def __init__(self, name):
        super(PlaceRecovery, self).__init__(name)

    def initialise(self):
        self._blackboard = py_trees.blackboard.Blackboard()
        self.scene = PlanningSceneInterface()

        # Check for return status of the end condition
        status_action_place = self._blackboard.get("status_action_place")
        if status_action_place is py_trees.common.Status.FAILURE:
            console.loginfo("Recovering from place failure")
            self.recover_action_place()

    def update(self):
        return py_trees.common.Status.SUCCESS

    def recover_action_place(self):
        # Open gripper and detach the object
        next_cube = self._blackboard.get("next_cube")
        block_manager = self._blackboard.get("block_manager")

        if next_cube is not None:
            left_right = next_cube.left_right
            open_gripper(left_right)
            self.scene.remove_attached_object()
            block_manager.remove_block(next_cube.id)
            self._blackboard.set("status_action_place","")