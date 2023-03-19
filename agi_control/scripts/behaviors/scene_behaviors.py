#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""This behavior is used to get the scene blocks from the scene and store them in the blackboard."""

# Rospy
import rospy
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped

# Moveit
from moveit_commander import PlanningSceneInterface, MoveGroupCommander
from moveit_msgs.srv import GetPlanningScene

# Pytrees
import py_trees
import py_trees.console as console
from py_trees.blackboard import Blackboard

# perseption
from agi_vision.msg import PerceptionMSG

# Utils
from utils.block import BlockManager
from utils.robot_utils import get_gripper_status, get_arm

import pdb
import time


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
        super(GetSceneBlocks, self).__init__(name=name)
        self.update_requested = False
        self.perception_data_list = []
        self.prev_perception_data = None

    def setup(self, timeout):

        self._scene = PlanningSceneInterface(synchronous=True)
        rospy.loginfo("Connecting to /get_planning_scene service")
        self.scene_srv = rospy.ServiceProxy("/get_planning_scene",
                                            GetPlanningScene)

        self.scene_srv.wait_for_service()

        rospy.sleep(1)

        self.block_manager = BlockManager()
        self._scene.clear()

        self.blackboard = Blackboard()
        # Add table to the planning scene
        table = {"size": [0.60, 0.75, 0.48]}
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base_footprint"
        table_pose.pose.position.x = 0.5
        table_pose.pose.position.y = 0
        table_pose.pose.position.z = table["size"][2] / 2
        table_pose.pose.orientation.w = 1.0
        self._scene.add_box("table", table_pose, table["size"])
        rospy.sleep(0.5)

        try:
            self.table_co = self._scene.get_objects(["table"])["table"]
            return True
        except KeyError:
            console.logerror("Table not found in the planning scene")
            return False

    def initialise(self):
        """
        Retrieve the collision objects from the planning scene, filter them to select only the ones that have an id that starts with 'box',
        and store them in the blackboard.

        Returns:
            None
        """
        # Get the perception data
        perception_data = self._get_published_perception()
        console.loginfo("Read perception data")

        # Create cubes from perception data
        self.block_manager.invalidate_blocks()
        for i, pose in enumerate(perception_data.pose_array.poses):
            pose_stamp = PoseStamped()
            pose_stamp.header.frame_id = "base_footprint"
            pose_stamp.pose = pose

            self.block_manager.manage(pose_stamp,
                                      confidence=perception_data.confidence[i],
                                      color=perception_data.color_names[i])

        self.block_manager.remove_invalid_blocks()

        self.__clean_scene()

        console.loginfo("Updated scene blocks")

        cubes = self.block_manager.get_blocks()

        # Store the cubes in the blackboard
        self.blackboard.set("scene_cubes", cubes)

    def update(self):

        return py_trees.common.Status.SUCCESS

    @staticmethod
    def _get_published_perception():
        """Retrieves the pose, color, ... published in '/PerceptionMSG' topic."""
        perception_data = rospy.wait_for_message('/PerceptionMSG',
                                                 PerceptionMSG)
        return perception_data

    def __clean_scene(self):
        """Removes all cubes from the scene that have no entry in the block manager."""
        scene_objects = self._scene.get_known_object_names()
        scene_objects = [obj for obj in scene_objects if obj.startswith('box')]
        block_ids = [block.id for block in self.block_manager.get_blocks()]
        for obj in scene_objects:
            if obj not in block_ids:
                console.loginfo(
                    "Removing object '{}' from scene as there is no cube manager correspondence"
                    .format(obj))
                self._scene.remove_world_object(obj)

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
        self.blackboard.set("next_block", None)
        return py_trees.common.Status.SUCCESS
