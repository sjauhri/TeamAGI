#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""This behavior is used to get the scene blocks from the scene and store them in the blackboard."""

# Rospy
import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped

# Moveit
from moveit_commander import PlanningSceneInterface
from moveit_msgs.srv import GetPlanningScene

# Pytrees
import py_trees
import py_trees.console as console
from py_trees.blackboard import Blackboard

# Utils
from utils.block import Block
from utils.block import BlockManager

import pdb


class GetSceneBlocks(py_trees.behaviour.Behaviour):
    """
    Retrieves the pose of each block in the scene, generates a collision object for each block,
    and stores them in the blackboard.
    """

    def __init__(self, name="GetSceneBlocks"):
        """
        Initialize the behavior.

        Args:
            name (str): The name of the behavior
        """
        self.blackboard = Blackboard()
        self._scene = PlanningSceneInterface()
        rospy.loginfo("Connecting to /get_planning_scene service")
        self.scene_srv = rospy.ServiceProxy("/get_planning_scene",
                                            GetPlanningScene)
        self.scene_srv.wait_for_service()

        rospy.sleep(1)
        super(GetSceneBlocks, self).__init__(name=name)

    def setup(self, timeout):
        self.block_manager = BlockManager()
        self._scene.clear()
        # Add table to the planning scene
        table = {"size": [0.60, 0.75, 0.45]}
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base_footprint"
        table_pose.pose.position.x = 0.5
        table_pose.pose.position.y = 0
        table_pose.pose.position.z = table["size"][2] / 2
        table_pose.pose.orientation.w = 1.0
        self._scene.add_box("table", table_pose, table["size"])
        try:
            self.table_co = self._scene.get_objects(["table"])["table"]
            return True
        except KeyError:
            console.logerr("Table not found in the planning scene")
            return False

    def update(self):
        """
        Retrieve the collision objects from the planning scene, filter them to select only the ones that have an id that starts with 'box',
        and store them in the blackboard.

        Returns:
            py_trees.common.Status.SUCCESS
        """
        block_detections = self.__get_published_poses()
        for pose in block_detections.poses:
            pose_stamp = PoseStamped()
            pose_stamp.header.frame_id = "base_footprint"
            pose_stamp.pose = pose
            self.block_manager.manage(pose_stamp)

        console.loginfo("Found {} blocks in the scene".format(
            len(block_detections.poses)))

        self.__update_contact_objects()

        blocks = self.block_manager.get_blocks()
        stack_blocks = self.blackboard.blocks_in_stack
        # Remove the blocks that are already in the stack
        if len(stack_blocks) > 0:
            blocks = [
                block for block in blocks if block.id not in
                [stack_block.id for stack_block in stack_blocks]
            ]

        self.blackboard.scene_blocks = blocks
        self.blackboard.table_block = self.table_co

        return py_trees.common.Status.SUCCESS

    def __get_published_poses(self):
        """Retrieves the pose published in '/cube_poses' topic."""
        cube_poses = rospy.wait_for_message('/cube_poses', PoseArray)
        return cube_poses

    def __update_contact_objects(self):

        # Get current param ~links_to_allow_contact

        links_to_allow_contact = rospy.get_param("~links_to_allow_contact", [])

        # Remove all objects from the list that have the name 'box'
        links_to_allow_contact = [
            link for link in links_to_allow_contact
            if not link.startswith('box')
        ]

        # Add the blocks to the list
        links_to_allow_contact.extend(
            [block.id for block in self.block_manager.get_blocks()])

        # Update the param
        rospy.set_param("~links_to_allow_contact", links_to_allow_contact)


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
        table_height = self.blackboard.table_block.primitives[0].dimensions[2]
        block_height = self.blackboard.next_block.collision_object.primitives[
            0].dimensions[2]
        place_offset = 0.035
        loc = PoseStamped()
        loc.header.frame_id = "base_footprint"
        loc.pose.position.x = 0.4
        loc.pose.position.y = 0.2
        loc.pose.position.z = 0.45 + 0.045 * len(
            self.blackboard.blocks_in_stack) + place_offset
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
