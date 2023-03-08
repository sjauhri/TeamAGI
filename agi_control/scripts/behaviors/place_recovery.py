#! /usr/bin/env python
# -*- coding: utf-8 -*-

import py_trees
from utils.robot_utils import recover_from_error

class PlaceRecovery(py_trees.behaviour.Behaviour):
    """This class can be used to create a recovery behaviour for the place subtask.

    Example:
        >>> recovery = PlaceRecovery("Recovery")
    """

    def __init__(self, name):
        super(PlaceRecovery, self).__init__(name)

    def initialise(self):
        self._blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        # Retrieve place_errors dictionary from blackboard
        place_errors = self._blackboard.get("place_errors")
        if place_errors is None:
            return py_trees.common.Status.SUCCESS
        
        # to do: set the error and recover of place
        for key, value in place_errors.items():
            if key == "some_error":
                res = recover_from_error(value)
                if res == py_trees.common.Status.FAILURE:
                    return py_trees.common.Status.FAILURE

        return py_trees.common.Status.SUCCESS
