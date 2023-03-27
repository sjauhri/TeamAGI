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
import numpy as np
import random
import math


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


class ShapePolicy(ActionPolicy):
    """Base class for action policies. Place cubes into different shapes.
    
    Args:
        name (str): The name of the policy
    """

    def __init__(self):
        super(ShapePolicy, self).__init__("ShapePolicy")
        console.loginfo("Initializing ShapePolicy")
        self._blackboard.set("cubes_in_shape", [])
        self._blackboard.set("cubes_in_center", [])
        self.initialized = False
        # table
        self.margin = 0.06
        self.table_x_min, self.table_x_max = (0.2 + self.margin,
                                              0.8 - self.margin)
        self.table_y_min, self.table_y_max = (-0.375 + self.margin,
                                              0.375 - self.margin)
        self.center_x = (self.table_x_min + self.table_x_max) / 2
        self.center_y = (self.table_y_min + self.table_y_max) / 2
        self.half_width = (self.table_x_max - self.table_x_min) / 2
        self.half_height = (self.table_y_max - self.table_y_min) / 2

    def update(self):
        self._scene_cubes = self._blackboard.get("scene_cubes")
        # Initialize scene cubes properties
        for scene_cube in self._scene_cubes:
            # Set properties if they have not been set
            if "pickable" not in scene_cube.properties:
                scene_cube.properties["pickable"] = True
            if "placed" not in scene_cube.properties:
                scene_cube.properties["placed"] = False
            # set xy rotation to zero, based on the prior knowledge that all squares lie flat on the table
            scene_cube.pose.pose.orientation.x = 0
            scene_cube.pose.pose.orientation.y = 0
            # Whether the coordinates are in quarter of the center of the desktop
            if self.center_x - self.half_width/4 <= scene_cube.pose.pose.position.x <= self.center_x + self.half_width/4 \
            and self.center_y - self.half_height/4 <= scene_cube.pose.pose.position.y <= self.center_y + self.half_height/4:
                scene_cube.properties["in_center"] = True
                self._blackboard.get("cubes_in_center").append(scene_cube)
            else:
                scene_cube.properties["in_center"] = False

            console.logdebug("test-------------------------------------test")
            print(scene_cube.properties["in_center"])

        if not self.initialized:
            self.initialize_shape()
            self.initialized = True

    def get_new_coordinate(self, x, y):
        # Check if the input coordinate is within the specified area
        if self.center_x - self.half_width/4 <= x <= self.center_x + self.half_width/4 \
            and self.center_y - self.half_height/4 <= y <= self.center_y + self.half_height/4:

            # If the input coordinate is within the specified area,
            # generate a new coordinate outside the area
            if x <= self.center_x:
                new_x = self.center_x + self.half_width - self.margin * 2
            else:
                new_x = self.center_x - self.half_width + self.margin * 2

            if y <= self.center_y:
                new_y = self.center_y + self.half_height - self.margin * 2
            else:
                new_y = self.center_y - self.half_height + self.margin * 2
            return (new_x, new_y)

        # If the input coordinate is already outside the specified area
        else:
            return (x, y)

    def get_target_location(self, next_cube=None):
        # Get the current top cube in the cube stack
        # if next_cube is None or len(
        #         self._blackboard.get("cubes_in_shape")) == 0:
        #     return None
        # else:
        #     top_cube = self._blackboard.get("cubes_in_shape")[-1]
        #     target_pose = PoseStamped()
        #     target_pose.header.frame_id = "base_footprint"
        #     target_pose.pose.position.x = top_cube.pose.pose.position.x
        #     target_pose.pose.position.y = top_cube.pose.pose.position.y
        #     target_pose.pose.position.z = top_cube.pose.pose.position.z + 0.06
        #     target_pose.pose.orientation.x = next_cube.pose.pose.orientation.x
        #     target_pose.pose.orientation.y = next_cube.pose.pose.orientation.y
        #     target_pose.pose.orientation.z = next_cube.pose.pose.orientation.z
        #     target_pose.pose.orientation.w = next_cube.pose.pose.orientation.w
        #     return target_pose

        # Clean up the center area of the desktop
        if next_cube is None or len(
                self._blackboard.get("cubes_in_shape")) == 0:
            return None
        else:
            cube = self._blackboard.get("cubes_in_shape")[-1]
            target_pose = PoseStamped()
            target_pose.header.frame_id = "base_footprint"
            # target_pose.pose.position.x = cube.pose.pose.position.x
            # target_pose.pose.position.y = cube.pose.pose.position.y
            target_pose.pose.position.x, target_pose.pose.position.y = self.get_new_coordinate(
                cube.pose.pose.position.x, cube.pose.pose.position.y)
            target_pose.pose.position.z = cube.pose.pose.position.z + 0.01
            target_pose.pose.orientation.x = 0
            target_pose.pose.orientation.y = 0
            target_pose.pose.orientation.z = 0
            target_pose.pose.orientation.w = next_cube.pose.pose.orientation.w
            return target_pose

    def get_next_cube(self):
        # Get scene cubes that are not placed
        pdb.set_trace()
        scene_cubes_not_placed = [
            scene_cubes for scene_cubes in self._scene_cubes
            if scene_cubes.properties["placed"] == False
        ]
        # Get scene cubes that are in the center
        scene_cubes_in_center = [
            scene_cubes for scene_cubes in scene_cubes_not_placed
            if scene_cubes.properties["in_center"] == True
        ]
        # Get scene cubes that are pickable
        scene_cubes_pickable = [
            scene_cubes for scene_cubes in scene_cubes_in_center
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

    def initialize_shape(self):
        # Initialize the stack with the cube having the highest confidence
        init_cube = self.get_next_cube()
        self._blackboard.get("cubes_in_shape").append(init_cube)
        init_cube.properties["placed"] = True
        init_cube.properties["fixed"] = True
        # init_cube.properties["in_center"] = False


class GetCubeBehaviourInShape(py_trees.behaviour.Behaviour):
    """Behaviour for getting a cube.
    
    Args:
        name (str): The name of the behaviour
    """

    def __init__(self, name):
        super(GetCubeBehaviourInShape, self).__init__(name)

    def setup(self, timeout):

        self._blackboard = py_trees.blackboard.Blackboard()
        self._action_policy = ShapePolicy()
        self._blackboard.set("get_cube_policy", self._action_policy)

        rospy.logdebug("Initialize GetCubeBehaviourInShape")
        super(GetCubeBehaviourInShape, self).setup(timeout)

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
