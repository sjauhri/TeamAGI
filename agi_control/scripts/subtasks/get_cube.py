#! /usr/bin/env python
# -*- coding: utf-8 -*-

import py_trees
import operator

from subtasks.subtask import Subtask
from subtasks.subtask import SubtaskGuard

task_name = "Get Cube"

blackboard = py_trees.blackboard.Blackboard()

#============================================================================
# Guards
# Define conditions that must be met before the subtask can be executed.
#
# (IMPORTANT): If the guard succeeds, the subtask will not be executed.
#              If the guard fails, the subtask will be executed.
#============================================================================

# Check if the variable next_block is not None.
# If the variable next_block is not None, the guard will succeed and the subtask will not be executed.
guard_has_block = py_trees.blackboard.CheckBlackboardVariable(
    name="Has Block",
    variable_name="next_block",
    expected_value='',
    comparison_operator=operator.ne)

# Check if there are no blocks in the scene by checking the variable scene_blocks
# If there are no blocks in the scene, the guard will succeed and the subtask will not be executed.
guard_has_no_scene_blocks = py_trees.blackboard.CheckBlackboardVariable(
    name="Has Scene Blocks",
    variable_name="scene_blocks",
    expected_value='',
    comparison_operator=operator.eq)

#============================================================================
# Actions
# Define the action behaviour of the subtask.
# If the action fails, the subtask will go to the recovery behaviour.
#============================================================================

#TODO: Add functionality to get the next block from the scene based on some criteria

#============================================================================
# End conditions
# Define the end condition behaviour of the subtask.
# If the end condition succeeds, the subtask will be finished.
# If the end condition fails, the subtask will go to the recovery behaviour.
#============================================================================
# Check if the variable next_block is not None
end_condition_has_block = guard_has_block

#============================================================================
# Recovery
# Define the recovery behaviour of the subtask.
# This should be a behaviour that can recover from a failure and will not fail
# in the general case.
#============================================================================

#TODO: Add functionality to recover from a failure


def create_get_cube_subtask():
    """Create a subtask for the task get cube.

    Returns:
        py_trees.composites.Sequence: Sequence of the subtask.
    """

    # Create a guard that is composed of multiple guards
    guard = SubtaskGuard("Guard")
    guard.add_guard(guard_has_block)
    guard.add_guard(guard_has_no_scene_blocks)

    get_cube_subtask = Subtask(task_name, guard, None, end_condition_has_block,
                               None)
    return get_cube_subtask.create_subtask()