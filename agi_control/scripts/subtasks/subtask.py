#! /usr/bin/env python
# -*- coding: utf-8 -*-

import py_trees
import rospy
import pdb


class SubtaskGuard(py_trees.behaviour.Behaviour):
    """This class can be used to create a guard that is composed of multiple guards.

    Example:
        >>> guard1 = py_trees.blackboard.CheckBlackboardVariable(
        ...     name="Guard1",
        ...     variable_name="next_cube",
        ...     expected_value='',
        ...     comparison_operator=operator.ne)
        >>> guard2 = py_trees.blackboard.CheckBlackboardVariable(
        ...     name="Guard2",
        ...     variable_name="next_cube",
        ...     expected_value='',
        ...     comparison_operator=operator.ne)
        >>> guard = SubtaskGuard("Guard")
        >>> guard.add_guard(guard1)
        >>> guard.add_guard(guard2)
    """

    def __init__(self, name):
        super(SubtaskGuard, self).__init__(name)
        self._guards = []

    def initialise(self):
        self._blackboard = py_trees.blackboard.Blackboard()

    def add_guard(self, guard):
        self._guards.append(guard)

    def update(self):
        for guard in self._guards:
            res = guard.update()
            if res != py_trees.common.Status.SUCCESS:
                rospy.loginfo("Guard {} failed.".format(guard.name))
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
        initial_sequence.add_child(self.initial_condition)
        initial_sequence.add_child(action_sequence)
        action_sequence.add_child(execute_action_sequence)
        action_sequence.add_child(end_condition_sequence)
        execute_action_sequence.add_child(self.action_guard)
        execute_action_sequence.add_child(self.action)
        end_condition_sequence.add_child(self.end_condition)

        #Recovery
        recovery_sequence.add_child(self.recovery_guard)

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
