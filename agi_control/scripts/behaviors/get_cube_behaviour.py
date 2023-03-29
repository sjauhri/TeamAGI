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
import sys
from math import sqrt


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
        # Get the position and orientation of the next cube
        next_orient = next_cube.pose.pose.orientation

        # Hardcode place position

        # For the first cube, find a free region
        if not self._blackboard.get("cubes_in_stack"):
            target = self._scene_utils.get_free_region()
            x = target[0]
            y = target[1]
            z = target[2]
        # For the rest of the cubes, stack them on top of the previous cube
        else:
            top_cube = self._blackboard.get("cubes_in_stack")[-1]
            target_pos = top_cube.pose.pose.position
            x = target_pos.x
            y = target_pos.y
            z = target_pos.z + 0.060

        # Set the target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_footprint"
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = z
        target_pose.pose.orientation = next_orient

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
        blockmanager = self._blackboard.get("block_manager")

        blockmanager.fit_neighbors()
        next_cube = self._action_policy.get_next_cube()

        # Get the target location for the next cube
        target_location = self._action_policy.get_target_location(next_cube)
        if next_cube is None or target_location is None:
            return py_trees.common.Status.FAILURE
        else:
            self._blackboard.set("next_cube", next_cube)
            self._blackboard.set("target_location", target_location)

            # inflate blocks to avoid collision
            blockmanager.fat_neighbors()
            return py_trees.common.Status.SUCCESS


class CleanCenterPolicy(ActionPolicy):
    """Base class for action policies. Clean the cubes on the center of the table.
    
    Args:
        name (str): The name of the policy
    """

    def __init__(self):
        super(CleanCenterPolicy, self).__init__("CleanCenterPolicy")
        console.loginfo("Initializing CleanCenterPolicy")
        # self._blackboard.set("cubes_in_stack", [])
        self._blackboard.set("cubes_in_center", [])
        self._blackboard.set("cubes_not_in_center", [])
        self.initialized = False
        # table
        self.margin = 0.06
        self.table_x_min, self.table_x_max = (0.2 + self.margin,
                                              0.8 - self.margin)
        self.table_y_min, self.table_y_max = (-0.375 + self.margin,
                                              0.375 - self.margin)
        self.table_center = ((self.table_x_min + self.table_x_max) / 2,
                             (self.table_y_min + self.table_y_max) / 2)
        self.half_width = (self.table_x_max - self.table_x_min) / 2
        self.half_length = (self.table_y_max - self.table_y_min) / 2
        self.center_x_min = self.table_center[0] - self.half_width / 4
        self.center_x_max = self.table_center[0] + self.half_width / 4
        self.center_y_min = self.table_center[1] - self.half_length / 4
        self.center_y_max = self.table_center[1] + self.half_length / 4

    def center_area(self, cubePose):
        if self.center_x_min <= cubePose.pose.pose.position.x <=  self.center_x_max \
            and self.center_y_min <= cubePose.pose.pose.position.y <= self.center_y_max:
            return True
        else:
            return False

    def distance(self, a, b):
        return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


    def get_place_position(self, table_center, x_min1, x_max_1, x_min2, x_max_2, \
                           y_min1, y_max_1, y_min2, y_max_2, cubes_not_in_center, min_distance=0.12, num_attempts=1000):
        # Function to find the place position to the cube in the center
        place_position = None
        closest_distance = sys.float_info.max
        list_position = []
        for cube in cubes_not_in_center:
            list_position.append((cube.pose.pose.position.x, cube.pose.pose.position.y))

        # Try num_attempts times to find a suitable coordinate
        for _ in range(num_attempts):
            # Try num_attempts times to find a suitable coordinate
            x = random.choice([
                random.uniform(x_min1, x_max_1),
                random.uniform(x_min2, x_max_2)
            ])
            y = random.choice([
                random.uniform(y_min1, y_max_1),
                random.uniform(y_min2, y_max_2)
            ])
            candidate_coordinate = (x, y)

            # Check if the candidate coordinate is at least min_distance away from all existing coordinates in list_position
            if all(
                    self.distance(candidate_coordinate, position) >=
                    min_distance for position in list_position):
                candidate_distance = self.distance(candidate_coordinate,
                                                   table_center)

                # If the candidate distance is less than the current closest distance, update the closest distance and coordinate
                if candidate_distance < closest_distance:
                    closest_distance = candidate_distance
                    place_position = candidate_coordinate

        return place_position

    def update(self):
        self._scene_cubes = self._blackboard.get("scene_cubes")
        self._center_cubes = self._blackboard.get("cubes_in_center")
        self._not_center_cubes = self._blackboard.get("cubes_not_in_center")
        # Initialize scene cubes properties
        for scene_cube in self._scene_cubes:
            # Set properties if they have not been set
            if "pickable" not in scene_cube.properties:
                scene_cube.properties["pickable"] = True
            if "in_stack" not in scene_cube.properties:
                scene_cube.properties["in_stack"] = False
            # set xy rotation to zero, based on the prior knowledge that all squares lie flat on the table
            scene_cube.pose.pose.orientation.x = 0
            scene_cube.pose.pose.orientation.y = 0
            # Whether the coordinates are in quarter of the center of the desktop
            if self.center_area(scene_cube):
                scene_cube.properties["in_center"] = True
                self._blackboard.get("cubes_in_center").append(scene_cube)
            else:
                scene_cube.properties["in_center"] = False
                self._blackboard.get("cubes_not_in_center").append(scene_cube)

        if not self.initialized:
            self.initialize()
            self.initialized = True

    def get_target_location(self, next_cube=None):
        # Clean up the center area of the desktop
        if next_cube is None or len(
                self._blackboard.get("cubes_in_center")) == 0:
            return None
        else:
            cube_center = self._blackboard.get("cubes_in_center")[-1]
            target_pose = PoseStamped()
            target_pose.header.frame_id = "base_footprint"
            # target_pose.pose.position.x = cube.pose.pose.position.x
            # target_pose.pose.position.y = cube.pose.pose.position.y
            target_pose.pose.position.x, target_pose.pose.position.y = \
            self.get_place_position(self.table_center,self.table_x_min,self.center_x_min,self.center_x_max,self.table_x_max,
                                    self.table_y_min,self.center_y_min,self.center_x_max,self.table_y_max,self._blackboard.get("cubes_not_in_center"))
            target_pose.pose.position.z = cube_center.pose.pose.position.z + 0.01
            target_pose.pose.orientation.x = 0
            target_pose.pose.orientation.y = 0
            target_pose.pose.orientation.z = 0
            target_pose.pose.orientation.w = next_cube.pose.pose.orientation.w

            return target_pose

    def get_next_cube(self):
        # Get scene cubes that are not in the stack
        scene_cubes_not_placed = [
            scene_cubes for scene_cubes in self._center_cubes
            if scene_cubes.properties["in_stack"] == False
        ]
        # Get scene cubes that are pickable
        scene_cubes_pickable = [
            scene_cubes for scene_cubes in scene_cubes_not_placed
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

    def initialize(self):
        # Initialize the stack with the cube having the highest confidence
        init_cube = self.get_next_cube()
        if init_cube !=None:
            self._blackboard.get("cubes_in_center").append(init_cube)
            # init_cube.properties["in_stack"] = True
            init_cube.properties["fixed"] = True
            init_cube.properties["in_center"] = False


class GetCubeBehaviour_CleanCenter(py_trees.behaviour.Behaviour):
    """Behaviour for getting a cube.
    
    Args:
        name (str): The name of the behaviour
    """

    def __init__(self, name):
        super(GetCubeBehaviour_CleanCenter, self).__init__(name)

    def setup(self, timeout):

        self._blackboard = py_trees.blackboard.Blackboard()
        self._action_policy = CleanCenterPolicy()
        self._blackboard.set("get_cube_policy", self._action_policy)

        rospy.logdebug("Initialize GetCubeBehaviour_CleanCenter")
        super(GetCubeBehaviour_CleanCenter, self).setup(timeout)

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
