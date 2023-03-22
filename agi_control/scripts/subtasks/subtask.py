#! /usr/bin/env python
# -*- coding: utf-8 -*-

import py_trees


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

    def __init__(self, name):
        self.name = name
        self.initial_condition = None
        self.action_guard = None
        self.action = None
        self.end_condition = None
        self.recovery_guard = None
        self.recovery = None

    def create_subtask(self):
        """This method creates a subtask.

        Returns:
            py_trees.composites.Sequence: Subtask.
        """

        subtask = py_trees.composites.Selector(name=self.name)

        #Composites
        initial_sequence = py_trees.composites.Sequence(
            name='Initial-{}'.format(self.name))
        recovery_sequence = py_trees.composites.Sequence(
            name='Recovery-{}'.format(self.name))
        action_sequence = py_trees.composites.Sequence(
            name='Action-{}'.format(self.name))
        execute_action_sequence = py_trees.composites.Sequence(
            name='Execute Action-{}'.format(self.name))
        end_condition_sequence = py_trees.composites.Sequence(
            name='End Condition-{}'.format(self.name))

        #Initial
        if self.initial_condition is not None:
            initial_sequence.add_child(self.initial_condition)
        initial_sequence.add_child(action_sequence)
        action_sequence.add_child(execute_action_sequence)
        action_sequence.add_child(end_condition_sequence)
        if self.action_guard is not None:
            execute_action_sequence.add_child(self.action_guard)
        if self.action is not None:
            execute_action_sequence.add_child(self.action)
        if self.end_condition is not None:
            end_condition_sequence.add_child(self.end_condition)

        #Recovery
        if self.recovery is not None:
            recovery_sequence.add_child(self.recovery)

        #Task
        subtask.add_child(initial_sequence)
        subtask.add_child(recovery_sequence)

        return subtask

    def set_initial_condition(self, initial_condition):
        self.initial_condition = initial_condition

    def set_action_guard(self, action_guard):
        self.action_guard = action_guard

    def set_action(self, action):
        self.action = action

    def set_end_condition(self, end_condition):
        self.end_condition = end_condition

    def set_recovery_guard(self, recovery_guard):
        self.recovery_guard = recovery_guard

    def set_recovery(self, recovery):
        self.recovery = recovery

    def __str__(self):
        return self.name
