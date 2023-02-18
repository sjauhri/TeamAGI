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
            console.logerror("Table not found in the planning scene")
            return False

    def update(self):
        """
        Retrieve the collision objects from the planning scene, filter them to select only the ones that have an id that starts with 'box',
        and store them in the blackboard.

        Returns:
            py_trees.common.Status.SUCCESS
        """
        cube_detections = self.__get_published_poses()
        for pose in cube_detections.poses:
            pose_stamp = PoseStamped()
            pose_stamp.header.frame_id = "base_footprint"
            pose_stamp.pose = pose
            # TODO: Add confidence to the block
            self.block_manager.manage(pose_stamp, confidence=1.0)

        console.loginfo("Found {} cubes in the scene".format(
            len(cube_detections.poses)))

        self.__update_contact_objects()

        cubes = self.block_manager.get_blocks()
        # stack_blocks = self.blackboard.blocks_in_stack
        # # Remove the blocks that are already in the stack
        # if len(stack_blocks) > 0:
        #     blocks = [
        #         block for block in blocks if block.id not in
        #         [stack_block.id for stack_block in stack_blocks]
        #     ]

        self.blackboard.scene_cubes = cubes
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


class ResetNextBlock(py_trees.behaviour.Behaviour):

    def __init__(self, name="ResetNextBlock"):
        """Initialize the behavior."""
        super(ResetNextBlock, self).__init__(name=name)
        self.blackboard = Blackboard()

    def update(self):
        """Reset the next block in the blackboard."""
        self.blackboard.next_cube = None
        return py_trees.common.Status.SUCCESS
