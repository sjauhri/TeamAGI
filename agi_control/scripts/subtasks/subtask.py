#! /usr/bin/env python
# -*- coding: utf-8 -*-

import py_trees
import py_trees_ros.trees


class SubtaskGuard(py_trees.behaviour.Behaviour):
    """This class can be used to create a guard that is composed of multiple guards.

    Example:
        >>> guard1 = py_trees.blackboard.CheckBlackboardVariable(
        ...     name="Guard1",
        ...     variable_name="next_block",
        ...     expected_value='',
        ...     comparison_operator=operator.ne)
        >>> guard2 = py_trees.blackboard.CheckBlackboardVariable(
        ...     name="Guard2",
        ...     variable_name="next_block",
        ...     expected_value='',
        ...     comparison_operator=operator.ne)
        >>> guard = SubtaskGuard("Guard")
        >>> guard.add_guard(guard1)
        >>> guard.add_guard(guard2)
    """

    def __init__(self, name):
        super(SubtaskGuard, self).__init__(name)
        self._guards = []

    def add_guard(self, guard):
        self._guards.append(guard)

    def update(self):
        for guard in self._guards:
            if guard():
                return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS


class Subtask:
    """This class can be used to create a subtask.

    Args:
        name (str): Name of the subtask.
        guard (py_trees.behaviour.Behaviour): Guard of the subtask.
        action (py_trees.behaviour.Behaviour): Action of the subtask.
        end_condition (py_trees.behaviour.Behaviour): End condition of the subtask.
        recovery (py_trees.behaviour.Behaviour): Recovery of the subtask.

    Example:
        >>> guard = SubtaskGuard("Guard")
        >>> action = py_trees.behaviours.Running(name="Action")
        >>> end_condition = py_trees.behaviours.Running(name="End Condition")
        >>> recovery = py_trees.behaviours.Running(name="Recovery")
        >>> subtask = Subtask("Subtask", guard, action, end_condition, recovery)
        >>> subtask.create_subtask()
    """

    def __init__(self, name, guard, action, end_condition, recovery):
        self.name = name
        self.guard = guard
        self.action = action
        self.end_condition = end_condition
        self.recovery = recovery

    def create_subtask(self):
        subtask = py_trees.composites.Selector(name=self.name)
        executor = py_trees.composites.Sequence(
            name="Execute_{}".format(self.name))
        subtask.add_child(self.guard)
        subtask.add_child(executor)
        subtask.add_child(self.recovery)
        executor.add_child(self.action)
        executor.add_child(self.end_condition)

        return subtask
