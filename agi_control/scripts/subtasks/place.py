#!/usr/bin/env python
# -*- coding: utf-8 -*-

# System imports
import pdb
import operator

# PyTrees imports
import py_trees

# AGI imports
from behaviors.place_behaviour import PlaceBehaviour
from behaviors.place_recovery import PlaceRecovery
from subtasks.subtask import Subtask

task_name = "Place"

blackboard = py_trees.blackboard.Blackboard()

#============================================================================
# Initial conditions
# Define conditions that must be met before the subtask can be executed.
#
# (IMPORTANT): If the guard succeeds, the subtask will not be executed.
#              If the guard fails, the subtask will be executed.
#============================================================================

# Check if the variable goal_cube exists.
# If the variable goal_cube is None, the guard will fail and the subtask will not be executed.
guard_has_cube = py_trees.blackboard.CheckBlackboardVariable(
    name="IC - Has Target Location | Place", variable_name="target_location")

#============================================================================
# Action Guards
# Define conditions that must be met before the action can be executed.
#
# (IMPORTANT): If the guard succeeds, the action will be executed.
#              If the guard fails, the action will not be executed.
#============================================================================

# Check if the gripper is closed.
# If the gripper is not closed, the guard will fail and the subtask will not be executed.
guard_gripper_has_cube = py_trees.blackboard.CheckBlackboardVariable(
    name="AG - Gripper With Cube | Place",
    variable_name="gripper_status",
    expected_value="with_cube",
    comparison_operator=operator.eq)

#============================================================================
# Actions
# Define the action behaviour of the subtask.
# If the action fails, the subtask will go to the recovery behaviour.
#============================================================================
action_behavior = PlaceBehaviour("Place")

#============================================================================
# End conditions
# Define the end condition behaviour of the subtask.
# If the end condition succeeds, the subtask will be finished.
# If the end condition fails, the subtask will go to the recovery behaviour.
#============================================================================
# Check if the cube_status is on_goal
end_condition_cube_on_goal = py_trees.blackboard.CheckBlackboardVariable(
    name="EC - Cube on Goal | Place",
    variable_name="cube_status",
    expected_value='on_goal',
    comparison_operator=operator.eq)

#============================================================================
# Recovery
# Define the recovery behaviour of the subtask.
# If the action or end condition fails, the subtask will go to the recovery behaviour.
#============================================================================
recovery = PlaceRecovery("Place Recovery")


def create_place_subtask():
    """
    Create the place subtask.
    """

    # initial_condition = SubtaskGuard(name="Initial Condition - Place")
    # initial_condition.add_guard(guard_has_cube)

    # action_guard = SubtaskGuard(name="Action Guard - Place")
    # action_guard.add_guard(guard_gripper_closed)

    # end_condition = SubtaskGuard(name="End Condition - Place")
    # end_condition.add_guard(end_condition_cube_on_goal)

    # recovery_guard = SubtaskGuard(name="Recovery Guard - Place")

    initial_condition = py_trees.decorators.StatusToBlackboard(
        name="Initial Condition - Place",
        child=guard_has_cube,
        variable_name="status_ic_place")

    action_guard = py_trees.decorators.StatusToBlackboard(
        name="Action Guard - Place",
        child=guard_gripper_has_cube,
        variable_name="status_ag_place")

    end_condition = py_trees.decorators.StatusToBlackboard(
        name="End Condition - Place",
        child=end_condition_cube_on_goal,
        variable_name="status_ec_place")

    action = py_trees.decorators.StatusToBlackboard(
        name="Action - Place",
        child=action_behavior,
        variable_name="status_action_place")

    initial_condition = guard_has_cube
    action_guard = guard_gripper_has_cube
    end_condition = end_condition_cube_on_goal

    place_subtask = Subtask(name=task_name)
    place_subtask.set_action(action)
    place_subtask.set_initial_condition(initial_condition)
    place_subtask.set_action_guard(action_guard)
    place_subtask.set_end_condition(end_condition)
    place_subtask.set_recovery(recovery)

    return place_subtask.create_subtask()
