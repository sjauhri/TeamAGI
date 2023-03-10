#! /usr/bin/env python
# -*- coding: utf-8 -*-

from subtasks.subtask import Subtask
from subtasks.subtask import SubtaskGuard
from behaviors.get_goal_behaviour import GetGoalBehaviour
import py_trees
import operator

task_name = "Get Goal"

blackboard = py_trees.blackboard.Blackboard()

#============================================================================
# Initial conditions
# Define conditions that must be met before the action can be executed.
#
# (IMPORTANT): If the guards succeed, the subtask will be executed.
#              If the guards fail, the subtask will not be executed.
#============================================================================

# Check if there is a cube in the gripper by checking the variable cube_in_gripper
# If there is a cube in the gripper, the guard will succeed and the subtask will be executed.
guard_cube_in_gripper = py_trees.blackboard.CheckBlackboardVariable(
    name="IC - Cube in Gripper | Get Goal",
    variable_name="cube_in_gripper",
    expected_value=True,
    comparison_operator=operator.eq)

# Check if the variable next_goal is not None.
# If the variable next_goal is not empty, the guard will succeed and the subtask will be executed.
guard_has_goal = py_trees.blackboard.CheckBlackboardVariable(
    name="IC - Has Goal | Get Goal", variable_name="next_goal")

#============================================================================
# Actions
# Define the action behaviour of the subtask.
# If the action fails, the subtask will go to the recovery behaviour.
#============================================================================

action = GetGoalBehaviour("Get Goal")

#============================================================================
# End conditions
# Define the end condition behaviour of the subtask.
# If the end condition succeeds, the subtask will be finished.
# If the end condition fails, the subtask will go to the recovery behaviour.
#============================================================================
# Check if the variable next_goal is not None
end_condition_has_goal = py_trees.blackboard.CheckBlackboardVariable(
    name="EC - Has Goal | Get Goal",
    variable_name="next_goal",
    expected_value='',
    comparison_operator=operator.ne)

#============================================================================
# Recovery
# Define the recovery behaviour of the subtask.
# This should be a behaviour that can recover from a failure and will not fail
# in the general case.
#============================================================================

# TODO: Add functionality to recover from a failure


def create_get_goal_subtask():
    """Create a subtask for the task get goal.

    Returns:
        py_trees.composites.Sequence: Sequence of the subtask.
    """

    # Create a guard that is composed of multiple guards
    initial_condition = SubtaskGuard("Initial Condition - Get Goal")
    initial_condition.add_guard(guard_cube_in_gripper)
    initial_condition.add_guard(guard_has_goal)

    action_guard = SubtaskGuard("Action Guard - Get Goal")

    end_condition = SubtaskGuard("End Condition - Get Goal")
    end_condition.add_guard(end_condition_has_goal)

    recovery_guard = SubtaskGuard("Recovery Guard - Get Goal")

    get_goal_subtask = Subtask(name=task_name)
    get_goal_subtask.set_action(action)
    get_goal_subtask.set_initial_condition(initial_condition)
    get_goal_subtask.set_action_guard(action_guard)
    get_goal_subtask.set_end_condition(end_condition)
    get_goal_subtask.set_recovery_guard(recovery_guard)

    return get_goal_subtask.create_subtask()
