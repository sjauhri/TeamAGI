#! /usr/bin/env python
# -*- coding: utf-8 -*-

import pdb
import py_trees
import operator

from subtasks.subtask import Subtask
from behaviors.get_cube_behaviour import GetCubeBehaviour,GetCubeBehaviour_CleanCenter
task_name = "Get Cube"

blackboard = py_trees.blackboard.Blackboard()

#============================================================================
# Initial conditions
# Define conditions that must be met before the action can be executed.
#
# (IMPORTANT): If the guards succeed, the subtask will be executed.
#              If the guards fail, the subtask will not be executed.
#============================================================================

# Check if there are cubes in the scene by checking the variable scene_blocks
# If there are cubes in the scene, the guard will succeed and the subtask will be executed.
guard_has_no_scene_cube = py_trees.blackboard.CheckBlackboardVariable(
    name="IC - Has No Scene Cube | Get Cube",
    variable_name="scene_cubes",
    expected_value='',
    comparison_operator=operator.ne)

# Check if the variable next_cube is None.
# If the variable next_cube is empty, the guard will succeed and the subtask will be executed.
guard_has_cube = py_trees.decorators.FailureIsSuccess(
    py_trees.blackboard.CheckBlackboardVariable(
        name="IC - Has Next Cube | Get Cube", variable_name="next_cube"))

#============================================================================
# Actions
# Define the action behaviour of the subtask.
# If the action fails, the subtask will go to the recovery behaviour.
#============================================================================

action = GetCubeBehaviour("Get Cube")
# action = GetCubeBehaviour_CleanCenter(("Get Cube"))

#============================================================================
# End conditions
# Define the end condition behaviour of the subtask.
# If the end condition succeeds, the subtask will be finished.
# If the end condition fails, the subtask will go to the recovery behaviour.
#============================================================================
# Check if the variable next_cube is not None
end_condition_has_cube = py_trees.blackboard.CheckBlackboardVariable(
    name="EC - Has Next Cube | Get Cube",
    variable_name="next_cube",
    expected_value='',
    comparison_operator=operator.ne)

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

    # # Create a guard that is composed of multiple guards
    # initial_condition = SubtaskGuard("Initial Conditon - Get Cube")
    # initial_condition.add_guard(guard_has_cube)
    # initial_condition.add_guard(guard_has_no_scene_cube)

    # action_guard = SubtaskGuard("Action Guard - Get Cube")

    # end_condition = SubtaskGuard("End Condition - Get Cube")

    # recovery_guard = SubtaskGuard("Recovery Guard - Get Cube")

    initial_condition = py_trees.composites.Sequence(
        "Initial Condition - Get Cube")
    initial_condition_1 = py_trees.decorators.StatusToBlackboard(
        name="IC - Has Next Cube | Get Cube",
        child=guard_has_cube,
        variable_name="status_ic_has_next_cube_get_cube")
    initial_condition_2 = py_trees.decorators.StatusToBlackboard(
        name="IC - Has No Scene Cube | Get Cube",
        child=guard_has_no_scene_cube,
        variable_name="status_ic_has_no_scene_cube_get_cube")

    initial_condition.add_children([initial_condition_1, initial_condition_2])

    end_condition = py_trees.decorators.StatusToBlackboard(
        name="EC - Has Next Cube | Get Cube",
        child=end_condition_has_cube,
        variable_name="status_ec_has_next_cube_get_cube")

    get_cube_subtask = Subtask(name=task_name)
    get_cube_subtask.set_action(action)
    get_cube_subtask.set_initial_condition(initial_condition)
    # get_cube_subtask.set_action_guard(action_guard)
    get_cube_subtask.set_end_condition(end_condition)
    # get_cube_subtask.set_recovery_guard(recovery_guard)

    return get_cube_subtask.create_subtask()