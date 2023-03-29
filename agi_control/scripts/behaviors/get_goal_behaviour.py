#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import py_trees
from geometry_msgs.msg import PoseStamped


class ActionPolicy:
    """
    Base class for action policies.
    """
    def __init__(self, name):
        self._name = name
        self._blackboard = py_trees.blackboard.Blackboard()

    def get_target_location(self):
        pass

    def get_next_goal(self):
        pass

class StackGoalsActionPolicy(ActionPolicy):
    """
    Action policy for stacking goals.
    """
    def __init__(self):
        super(StackGoalsActionPolicy, self).__init__("StackGoalsActionPolicy")
        self._blackboard.set("goals_in_stack", [])
        self._scene_goals = self._blackboard.get("scene_goals")
        # Initialize scene goals properties
        for scene_goal in self._scene_goals:
            scene_goal.properties["placeable"] = True
            scene_goal.properties["in_stack"] = False

        self.initialize_stack()

    def get_target_location(self):
        # Get the current top goal in the goal stack
        if len(self._blackboard.get("goals_in_stack")) == 0:
            return None
        else:
            top_goal = self._blackboard.get("goals_in_stack")[-1]
            target_pose = PoseStamped()
            target_pose.pose.position.x = top_goal.pose.pose.position.x
            target_pose.pose.position.y = top_goal.pose.pose.position.y
            #TODO Adjust height of stacking
            target_pose.pose.position.z = top_goal.pose.pose.position.z + 0.045
            return target_pose

    def get_next_goal(self):
        # Get scene goals that are not in the stack
        scene_goals_not_in_stack = [
            scene_goal for scene_goal in self._scene_goals
            if scene_goal.properties["in_stack"] == False
        ]
        # Get scene goals that are pickable
        scene_goals_pickable = [
            scene_goal for scene_goal in scene_goals_not_in_stack
            if scene_goal.properties["placeable"] != False
        ]
        # Order scene goals by confidence property
        scene_goals_pickable.sort(key=lambda x: x.confidence, reverse=True)

        # Get the next goal to place
        # If there are no goals in the scene, return None
        if len(scene_goals_pickable) == 0:
            return None
        else:
            return scene_goals_pickable[0]

    def initialize_stack(self):
        # Initialize the stack with the goal having the highest confidence
        init_goal = self.get_next_goal()
        self._blackboard.get("goals_in_stack").append(init_goal)
        init_goal.properties["in_stack"] = True


class GetGoalBehaviour(py_trees.behaviour.Behaviour):
    """
    Behaviour for getting a goal.
    """
    def __init__(self, name):
        super(GetGoalBehaviour, self).__init__(name)

    def initialise(self):
        self._blackboard = py_trees.blackboard.Blackboard()
        self._action_policy = StackGoalsActionPolicy()
        self._blackboard.set("get_goal_policy", self._action_policy)
        super(GetGoalBehaviour, self).initialise()

    def update(self):
        # Get the next goal to place
        next_goal = self._action_policy.get_next_goal()
        if next_goal is None:
            return py_trees.common.Status.FAILURE
        else:
            self._blackboard.set("next_goal", next_goal)
            return py_trees.common.Status.SUCCESS
