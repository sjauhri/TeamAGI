#! /usr/bin/env python
# -*- coding: utf-8 -*-

import pdb
import py_trees
import operator

from subtasks.subtask import Subtask
from subtasks.subtask import SubtaskGuard

from behaviors.pick_behaviour import PickBehaviour
from behaviors.pick_recovery import PickRecovery

from utils.robot_utils import open_gripper

task_name = "Pick"

blackboard = py_trees.blackboard.Blackboard()

#============================================================================
# Guards
# Define conditions that must be met before the subtask can be executed.
#
# (IMPORTANT): If the guard succeeds, the subtask will not be executed.
#              If the guard fails, the subtask will be executed.
#============================================================================
# Check if the variable next_cube is not None.
# If the variable next_cube is not None, the guard will succeed and the subtask will not be executed.
guard_has_cube = py_trees.blackboard.CheckBlackboardVariable(
    name="Has Next Cube | Pick",
    variable_name="next_cube",
    expected_value='',
    comparison_operator=operator.eq)

guard_gripper_open = py_trees.blackboard.CheckBlackboardVariable(
    name="Gripper Open | Pick",
    variable_name="gripper_status",
    expected_value=1,
    comparison_operator=operator.eq)

#============================================================================
# Actions
# Define the action behaviour of the subtask.
# If the action fails, the subtask will go to the recovery behaviour.
#============================================================================
action = PickBehaviour("Pick")

#============================================================================
# End conditions
# Define the end condition behaviour of the subtask.
# If the end condition succeeds, the subtask will be finished.
# If the end condition fails, the subtask will go to the recovery behaviour.
#============================================================================
# TODO: Check if the block is in the gripper

#============================================================================
# Recovery
# Define the recovery behaviour of the subtask.
# If the action or end condition fails, the subtask will go to the recovery behaviour.
#============================================================================
recovery = PickRecovery("Pick Recovery")


def create_pick_subtask():
    """
    Create the pick subtask.
    """
    guard = SubtaskGuard(name="Guard - Pick")
    guard.add_guard(guard_has_cube)
    guard.add_guard(guard_gripper_open)

    pick_subtask = Subtask(name=task_name,
                           guard=guard,
                           action=action,
                           end_condition=None,
                           recovery=recovery)

    return pick_subtask.create_subtask()