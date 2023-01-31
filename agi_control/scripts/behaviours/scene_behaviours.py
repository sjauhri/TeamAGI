#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""This behavior is used to get the scene blocks from the scene and store them in the blackboard."""

import py_trees
import rospy
from geometry_msgs.msg import PoseStamped
import py_trees.console as console
from py_trees.blackboard import Blackboard
from moveit_msgs.srv import GetPlanningScene


class GetSceneBlocks(py_trees.behaviour.Behaviour):
    """
    GetSceneBlocks is a behavior that retrieves the collision objects of the planning scene, filters the objects
    to select only the ones that have an id that starts with 'box', and stores them in the blackboard.
    """

    def __init__(self, name="GetSceneBlocks"):
        """
        Initialize the behavior.

        Args:
            name (str): The name of the behavior
        """
        super(GetSceneBlocks, self).__init__(name=name)
        self.blackboard = Blackboard()

        self.scene_srv = rospy.ServiceProxy("/get_planning_scene",
                                            GetPlanningScene)
        self.scene_srv.wait_for_service()

    def update(self):
        """
        Retrieve the collision objects from the planning scene, filter them to select only the ones that have an id that starts with 'box',
        and store them in the blackboard.

        Returns:
            py_trees.common.Status.SUCCESS
        """
        scene = self.scene_srv().scene.world.collision_objects
        blocks = [obj for obj in scene if obj.id.startswith("box")]
        stack_blocks = self.blackboard.blocks_in_stack
        # Remove the blocks that are already in the stack
        if len(stack_blocks) > 0:
            blocks = [
                block for block in blocks if block.id not in
                [stack_block.id for stack_block in stack_blocks]
            ]

        self.blackboard.scene_blocks = blocks

        return py_trees.common.Status.SUCCESS


class PopNextBlock(py_trees.behaviour.Behaviour):

    def __init__(self, name="PopNextBlock"):
        """Initialize the behavior."""
        super(PopNextBlock, self).__init__(name=name)
        self.blackboard = Blackboard()

    def update(self):
        """Pop the next block from the scene blocks list and store it in the blackboard."""
        if self.blackboard.scene_blocks:
            self.blackboard.next_block = self.blackboard.scene_blocks.pop()
            console.loginfo("Poped block: {}".format(
                self.blackboard.next_block.id))

            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE


class GetNextStackLocation(py_trees.behaviour.Behaviour):

    def __init__(self, name="GetNextStackLocation"):
        """Initialize the behavior."""
        super(GetNextStackLocation, self).__init__(name=name)
        self.blackboard = Blackboard()
        self.blackboard.blocks_in_stack = []

    def initialise(self):
        """Initialize the behavior."""
        self.blackboard.next_stack_location = None

    def update(self):
        """Get the next stack location from the scene and store it in the blackboard."""
        loc = PoseStamped()
        loc.header.frame_id = "base_footprint"
        loc.pose.position.x = 0.6
        loc.pose.position.y = 0
        loc.pose.position.z = 0.45 + 0.045 * len(
            self.blackboard.blocks_in_stack) + 0.01
        loc.pose.orientation.w = 1.0
        self.blackboard.next_stack_location = loc
        return py_trees.common.Status.SUCCESS


class ResetNextBlock(py_trees.behaviour.Behaviour):

    def __init__(self, name="ResetNextBlock"):
        """Initialize the behavior."""
        super(ResetNextBlock, self).__init__(name=name)
        self.blackboard = Blackboard()

    def update(self):
        """Reset the next block in the blackboard."""
        self.blackboard.next_block = None
        return py_trees.common.Status.SUCCESS
