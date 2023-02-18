#! /usr/bin/env python
# -*- coding: utf-8 -*-

# py_trees
import py_trees
import py_trees.console as console
import py_trees_ros.trees

# tiago_dual_pick_place
from tiago_dual_pick_place.msg import PickUpObjectAction
from tiago_dual_pick_place.msg import PickUpObjectGoal


class PickBehaviour(py_trees_ros.actions.ActionClient):
    """Action Client for picking up a block
    
    This class is a wrapper around the ActionClient class from py_trees_ros.
    It generates the goal for the action client and sets the action spec.
    """

    def __init__(self, name="Pick Block"):
        """Constructor for PickBlock
        
        Args:
            name (str, optional): Name of the behavior. Defaults to "Pick Block

        """
        super(PickBehaviour, self).__init__(name=name,
                                            action_spec=PickUpObjectAction,
                                            action_namespace="/pickup_object")

    def initialise(self):
        self.action_goal = self.get_pick_up_goal()
        super(PickBehaviour, self).initialise()

    def get_pick_up_goal(self):
        """Generates the goal for the action client
        
        Returns:
            PickUpObjectGoal: The goal for the action client
        """
        console.loginfo("Getting pick up goal")
        blackboard = py_trees.blackboard.Blackboard()
        pick_up_goal = PickUpObjectGoal()
        pick_up_goal.left_right = 'left'
        next_cube = blackboard.get("next_cube")
        if next_cube is None:
            pick_up_goal.object_name = ''
        else:
            pick_up_goal.object_name = next_cube.id
        console.logdebug("Pick up goal: {}".format(pick_up_goal))
        return pick_up_goal
