#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import py_trees

from utils.robot_utils import open_gripper


class PickRecovery(py_trees.behaviour.Behaviour):
    """This class can be used to create a recovery behaviour for the pick subtask.

    Example:
        >>> recovery = PickRecovery("Recovery")
    """

    def __init__(self, name):
        super(PickRecovery, self).__init__(name)

    def initialise(self):
        self._blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        # Retrieve pick_errors dictionary from blackboard
        pick_errors = self._blackboard.get("pick_errors")
        if pick_errors is None:
            return py_trees.common.Status.SUCCESS
        for key, value in pick_errors.items():
            if key == "closed_gripper":
                res = self.recover_closed_gripper(value)
                if res == py_trees.common.Status.FAILURE:
                    return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS

    def recover_closed_gripper(self, arg):
        res = open_gripper(arg)
        return res