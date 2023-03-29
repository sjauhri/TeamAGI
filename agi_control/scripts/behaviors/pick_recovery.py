#! /usr/bin/env python
# -*- coding: utf-8 -*-

# PyTrees imports
import py_trees
import py_trees.console as console

# AGI imports
from utils.robot_utils import open_gripper

# ROS imports
import rospy

# MoveIt imports
from moveit_commander import PlanningSceneInterface


class PickRecovery(py_trees.behaviour.Behaviour):
    """This class can be used to create a recovery behaviour for the pick subtask.

    Example:
        >>> recovery = PickRecovery("Recovery")
    """

    def __init__(self, name):
        super(PickRecovery, self).__init__(name)

    def initialise(self):
        self._blackboard = py_trees.blackboard.Blackboard()
        self.scene = PlanningSceneInterface()

        status_ec_pick = self._blackboard.get("status_ec_pick")
        if status_ec_pick is py_trees.common.Status.FAILURE:
            console.loginfo("Recovering from pick end condition failure")
            self.recover_ec_pick()


        status_action_pick = self._blackboard.get('status_action_pick')
        if status_action_pick is py_trees.common.Status.FAILURE:
            console.loginfo("Recovering from pick action")
            self.recover_action_pick()


    def update(self):
        return py_trees.common.Status.SUCCESS

    def recover_ec_pick(self):
        # Open gripper and detach the object
        next_cube = self._blackboard.get("next_cube")
        block_manager = self._blackboard.get("block_manager")

        if next_cube is not None:
            left_right = next_cube.left_right
            open_gripper(left_right)
            self.scene.remove_attached_object()
            block_manager.remove_block(next_cube.id)
            self._blackboard.set("status_ec_pick","")

    def recover_action_pick(self):
        next_cube = self._blackboard.get("next_cube")

        if next_cube is not None:
            console.loginfo("Trying different arm next time")
            cube_left_right = next_cube.left_right
            if cube_left_right == 'left':
                next_cube.left_right = 'right'
            else:
                next_cube.left_right = 'left'
        
            self._blackboard.set("status_action_pick","")
            
