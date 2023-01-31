#! /usr/bin/env python
# -*- coding: utf-8 -*-

import pdb

# py_trees
import py_trees


class CompareTwoBlackboardVariables(py_trees.behaviour.Behaviour):
    """Compares two blackboard variables
    """

    def __init__(self,
                 name="Compare Two Blackboard Variables",
                 variable1="",
                 variable2=""):
        """Constructor for CompareTwoBlackboardVariables
        
        Args:
            name (str, optional): Name of the behavior. Defaults to "Compare Two Blackboard Variables".
            variable1 (str, optional): Name of the first variable. Defaults to "".
            variable2 (str, optional): Name of the second variable. Defaults to "".
        """
        super(CompareTwoBlackboardVariables, self).__init__(name=name)
        self.variable1 = variable1
        self.variable2 = variable2

    def initialise(self):
        """Initialise the behavior
        """
        self.logger.debug("%s.initialise()" % self.__class__.__name__)

    def update(self):
        """Update the behavior
        """
        self.logger.debug("%s.update()" % self.__class__.__name__)
        blackboard = py_trees.blackboard.Blackboard()
        variable1 = blackboard.get(self.variable1)
        variable2 = blackboard.get(self.variable2)
        if variable1 == variable2:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """Terminate the behavior
        """
        self.logger.debug("%s.terminate()[%s->%s]" %
                          (self.__class__.__name__, self.status, new_status))
