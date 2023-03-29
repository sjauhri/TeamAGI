#! /usr/bin/env python
# -*- coding: utf-8 -*-

# System imports
import pdb
import operator

# PyTrees imports
import py_trees

# AGI imports
from behaviors.pick_behaviour import PickBehaviour
from behaviors.pick_recovery import PickRecovery
from subtasks.subtask import Subtask

task_name = "Pick"

blackboard = py_trees.blackboard.Blackboard()

#============================================================================
# Initial conditions
# Define conditions that must be met before the subtask can be executed.
#
# (IMPORTANT): If the guard succeeds, the subtask will not be executed.
#              If the guard fails, the subtask will be executed.
#============================================================================

# Check if the variable next_cube exists.
# If the variable next_cube is None, the guard will fail and the subtask will not be executed.
guard_has_cube = py_trees.blackboard.CheckBlackboardVariable(
    name="IC - Has Next Cube | Pick", variable_name="next_cube")

#============================================================================
# Action Guards
# Define conditions that must be met before the action can be executed.
#
# (IMPORTANT): If the guard succeeds, the action will be executed.
#              If the guard fails, the action will not be executed.
#============================================================================

# Check if the gripper is open.
# If the gripper is not open, the guard will fail and the subtask will not be executed.
guard_gripper_open = py_trees.blackboard.CheckBlackboardVariable(
    name="AG - Gripper Open | Pick",
    variable_name="gripper_status",
    expected_value="open",
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
# Check if the gripper_status is with_cube
end_condition_cube_gripped = py_trees.blackboard.CheckBlackboardVariable(
    name="EC - Cube Gripped | Pick",
    variable_name="gripper_status",
    expected_value='with_cube',
    comparison_operator=operator.eq)

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

    # initial_condition = SubtaskGuard(name="Initial Condition - Pick")
    # initial_condition.add_guard(guard_has_cube)

    # action_guard = SubtaskGuard(name="Action Guard - Pick")
    # action_guard.add_guard(guard_gripper_open)

    # end_condition = SubtaskGuard(name="End Condition - Pick")

    # recovery_guard = SubtaskGuard(name="Recovery Guard - Pick")

    initial_condition = py_trees.decorators.StatusToBlackboard(
        name="Initial Condition - Pick",
        variable_name="status_ic_pick",
        child=guard_has_cube)

    action_guard = py_trees.decorators.StatusToBlackboard(
        name="Action Guard - Pick",
        variable_name="status_ag_pick",
        child=guard_gripper_open)

    end_condition = py_trees.decorators.StatusToBlackboard(
        name="End Condition - Pick",
        variable_name="status_ec_pick",
        child=end_condition_cube_gripped)

    pick_subtask = Subtask(name=task_name)
    pick_subtask.set_action(action)
    pick_subtask.set_initial_condition(initial_condition)
    pick_subtask.set_action_guard(action_guard)
    pick_subtask.set_end_condition(end_condition)
    pick_subtask.set_recovery(recovery)

    return pick_subtask.create_subtask()
