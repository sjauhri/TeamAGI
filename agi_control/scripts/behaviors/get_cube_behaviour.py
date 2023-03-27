#! /usr/bin/env python
# -*- coding: utf-8 -*-

# System imports
import pdb

# PyTrees imports
import py_trees
import py_trees.console as console

# ROS imports
import rospy
from geometry_msgs.msg import PoseStamped

# AGI imports
from utils.scene_utils import SceneUtils


class ActionPolicy(object):
    """Base class for action policies.
    
    Args:
        name (str): The name of the policy
    """

    def __init__(self, name):
        self._name = name
        self._blackboard = py_trees.blackboard.Blackboard()
        self._scene_utils = SceneUtils()

    def update(self):
        pass

    def get_target_location(self):
        pass

    def get_next_cube(self):
        pass


class StackCubesActionPolicy(ActionPolicy):
    """Action policy for stacking cubes.

    Args:
        name (str): The name of the policy
    """

    def __init__(self):
        super(StackCubesActionPolicy, self).__init__("StackCubesActionPolicy")
        console.loginfo("Initializing StackCubesActionPolicy")
        self._blackboard.set("cubes_in_stack", [])
        self.initialized = False

    def update(self):
        self._scene_cubes = self._blackboard.get("scene_cubes")
        # Initialize scene cubes properties
        for scene_cube in self._scene_cubes:
            # Set properties if they have not been set
            if "pickable" not in scene_cube.properties:
                scene_cube.properties["pickable"] = True
            if "in_stack" not in scene_cube.properties:
                scene_cube.properties["in_stack"] = False

        if not self.initialized:
            #self.initialize_stack()
            self.initialized = True

    def get_target_location(self, next_cube=None):
        # Get the current top cube in the cube stack
        if next_cube is None or len(
                self._blackboard.get("cubes_in_stack")) == 0:
            target_pose = self._scene_utils.get_free_region()
            return target_pose
        else:
            top_cube = self._blackboard.get("cubes_in_stack")[-1]
            target_pose = PoseStamped()
            target_pose.header.frame_id = "base_footprint"
            target_pose.pose.position.x = top_cube.pose.pose.position.x
            target_pose.pose.position.y = top_cube.pose.pose.position.y
            target_pose.pose.position.z = top_cube.pose.pose.position.z + 0.06
            target_pose.pose.orientation.x = next_cube.pose.pose.orientation.x
            target_pose.pose.orientation.y = next_cube.pose.pose.orientation.y
            target_pose.pose.orientation.z = next_cube.pose.pose.orientation.z
            target_pose.pose.orientation.w = next_cube.pose.pose.orientation.w
            return target_pose

    def get_next_cube(self):
        # Get scene cubes that are not in the stack
        scene_cubes_not_in_stack = [
            scene_cubes for scene_cubes in self._scene_cubes
            if scene_cubes.properties["in_stack"] == False
        ]
        # Get scene cubes that are pickable
        scene_cubes_pickable = [
            scene_cubes for scene_cubes in scene_cubes_not_in_stack
            if scene_cubes.properties["pickable"] != False
        ]
        # Order scene cubes by confidence property
        scene_cubes_pickable.sort(key=lambda x: x.confidence, reverse=True)

        # Get the next cube to pick
        # If there are no cubes in the scene, return None
        if len(scene_cubes_pickable) == 0:
            return None
        else:
            return scene_cubes_pickable[0]

    def initialize_stack(self):
        # Initialize the stack with the cube having the highest confidence
        init_cube = self.get_next_cube()
        self._blackboard.get("cubes_in_stack").append(init_cube)
        init_cube.properties["in_stack"] = True
        init_cube.properties["fixed"] = True


class GetCubeBehaviour(py_trees.behaviour.Behaviour):
    """Behaviour for getting a cube.
    
    Args:
        name (str): The name of the behaviour
    """

    def __init__(self, name):
        super(GetCubeBehaviour, self).__init__(name)

    def setup(self, timeout):

        self._blackboard = py_trees.blackboard.Blackboard()
        self._action_policy = StackCubesActionPolicy()
        self._blackboard.set("get_cube_policy", self._action_policy)

        rospy.logdebug("Initialize GetCubeBehaviour")
        super(GetCubeBehaviour, self).setup(timeout)

    def update(self):
        # Update the action policy
        self._action_policy.update()

        # Get the next cube to pick
        next_cube = self._action_policy.get_next_cube()

        # Get the target location for the next cube
        target_location = self._action_policy.get_target_location(next_cube)
        if next_cube is None or target_location is None:
            return py_trees.common.Status.FAILURE
        else:
            self._blackboard.set("next_cube", next_cube)
            self._blackboard.set("target_location", target_location)
            return py_trees.common.Status.SUCCESS
