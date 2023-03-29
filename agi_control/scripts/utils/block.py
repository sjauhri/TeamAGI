#! /usr/bin/env python
# -*- coding: utf-8 -*-

# ROS imports
from geometry_msgs.msg import PoseStamped
import rospy

# MoveIt imports=
from moveit_commander import PlanningSceneInterface

# AGI imports
from utils.robot_utils import get_arm

import py_trees


class Block():
    """This class represents a block in the scene.
    
    Args:
        id (str): The id of the block
        pose (PoseStamped): The pose of the block
    
    Example:
        >>> block = Block("box1", PoseStamped())
    """

    def __init__(self, id, pose, confidence, color=""):
        self._scene = PlanningSceneInterface()
        self._id = id
        self._pose = pose
        self._color = color
        self._scene.add_box(self._id, self._pose, (0.045, 0.045, 0.045))
        rospy.sleep(0.1)
        self._collision_object = self._scene.get_objects([self._id])[self._id]
        self._properties = {}
        self._confidence = confidence
        self._left_right = get_arm(self)
        self._invalid = False

    @property
    def id(self):
        return self._id

    @property
    def collision_object(self):
        return self._collision_object

    @property
    def pose(self):
        return self._pose

    @property
    def color(self):
        return self._color

    @property
    def position(self):
        return self._pose.pose.position

    @property
    def orientation(self):
        return self._pose.pose.orientation

    @property
    def properties(self):
        return self._properties

    @property
    def confidence(self):
        return self._confidence

    @property
    def left_right(self):
        return self._left_right

    @property
    def invalid(self):
        return self._invalid

    def add_property(self, name, value):
        self._properties[name] = value

    def update(self):
        try:
            self._scene.remove_world_object(self._id)
            self._scene.add_box(self._id, self._pose, (0.045, 0.045, 0.045))
            rospy.sleep(0.1)
            self._collision_object = self._scene.get_objects([self._id
                                                              ])[self._id]
        except:
            pass

    def inflate(self):
        size = (0.055, 0.055, 0.055)
        self.resize_block(size)

    def deflate(self):
        self.resize_block((0.045, 0.045, 0.045))

    def resize_block(self, size):
        try:
            print("Resize Block")
            self._scene.remove_world_object(self._id)
            self._scene.add_box(self._id, self._pose, size)
            rospy.sleep(0.1)
            self._collision_object = self._scene.get_objects([self._id
                                                                ])[self._id]
        except:
            pass


    def remove(self):
        self._scene.remove_world_object(self._id)


class BlockManager():
    """This class manages the blocks in the scene.

    Example:
        >>> block_manager = BlockManager()
        >>> if block_manager.identify_block(pose) is None:
        >>>     block_manager.add_block("box1", pose)
        >>> else:
        >>>     id = block_manager.identify_block(pose)
        >>>     block_manager.update_block(id, pose)

    """

    def __init__(self):
        self.next_id = 0
        self._blocks = {}
        self._blackboard = py_trees.blackboard.Blackboard()

    def add_block(self, id, pose, confidence, color=""):
        """This function adds a block to the block manager.
        Args:
            id (str): The id of the block
            pose (PoseStamped): The pose of the block
        """
        rospy.loginfo("Adding block " + id)
        self._blocks[id] = Block(id, pose, confidence, color)

    def update_block(self, id, pose, confidence):
        """This function updates the pose of a block.
        Args:
            id (str): The id of the block
            pose (PoseStamped): The new pose of the block
        """
        rospy.loginfo("Updating block " + id)
        block = self._blocks[id]
        # Fixed blocks are not updated
        if block.properties.get("fixed") == True:
            return

        # Continue updating the block
        block._pose = pose
        block._confidence = confidence
        block._invalid = False
        block.update()

    def identify_block(self, pose, threshold=0.04, color=""):
        """This function identifies the block that is closest to the given pose.
        If no block is close enough, then it returns None.
        Args:
            pose (PoseStamped): The pose of the block
        Returns:
            str: The id of the block
        """
        for block in self._blocks.values():
            if self._is_close(block.pose, pose, threshold):
                # checks color of identified block, when a proper one is given
                # if color not in ["red", "blue", "yellow", "green"
                #                  ] or block.color == color:
                return block.id
        return None

    def _is_close(self, pose1, pose2, threshold):
        """This function checks if two poses are close enough.
        Args:
            pose1 (PoseStamped): The first pose
            pose2 (PoseStamped): The second pose
            threshold (float): The threshold distance
        Returns:
            bool: True if the poses are close enough, False otherwise
        """
        return self._distance(pose1, pose2) < threshold

    def _distance(self, pose1, pose2):
        """This function calculates the distance between two poses.
        Args:
            pose1 (PoseStamped): The first pose
            pose2 (PoseStamped): The second pose
        Returns:
            float: The distance between the two poses
        """
        return ((pose1.pose.position.x - pose2.pose.position.x)**2 +
                (pose1.pose.position.y - pose2.pose.position.y)**2 +
                (pose1.pose.position.z - pose2.pose.position.z)**2)**0.5

    def get_block(self, id):
        """This function returns the block with the given id.
        Args:
            id (str): The id of the block
        Returns:
            Block: The block with the given id
        """
        return self._blocks[id]

    def get_blocks(self, ids=None):
        """This function returns the blocks with the given ids.
        Args:
            ids (list): A list of ids
        Returns:
            list: A list of blocks
        """
        if ids is None:
            return self._blocks.values()
        else:
            return [self._blocks[id] for id in ids]

    def invalidate_blocks(self):
        """This function invalidates all blocks. Except the blocks that are fixed
        """
        for block in [
                block for block in self._blocks.values()
                if not block.properties.get("fixed", False)
        ]:
            block._invalid = True

    def manage(self, pose, confidence, color):
        """This function manages the blocks in the scene.
        First, it invalidates all blocks. Then, it checks if the given pose
        corresponds to an existing block. If so, it updates the block.
        Otherwise, it adds a new block. A block is considered valid if it is
        updated or added.
        Finally, it removes all invalid blocks.

        Args:
            pose (PoseStamped): The pose of the block
        """
        rospy.loginfo("Managing blocks")
        # check if the block is already in the scene
        id = self.identify_block(pose, color=color)
        if id is None:
            id = "box" + str(self.next_id)
            self.add_block(id, pose, confidence, color)
            self.next_id += 1
        else:
            self.update_block(id, pose, confidence)

        return id

    def remove_invalid_blocks(self):
        """This function removes all invalid blocks.
        And blocks that are not part of the block manager.
        """
        # remove invalid blocks
        for id, block in self._blocks.items():
            if block._invalid:
                rospy.loginfo("Removing block " + id + " (invalid)")
                self.remove_block(id)

    def remove_block(self, id):
        """This function removes a block from the block manager.
        Args:
            id (str): The id of the block
        """
        rospy.loginfo("Removing block " + id)
        if id in self._blocks:
            block = self._blocks[id]
            block.remove()
            self._blocks.pop(id)

    def fat_neighbors(self):
        for block in self._blocks.values():
            pick_block = self._blackboard.get("next_cube")
            if pick_block is None:
                return
            if not block.properties.get("in_stack"):
                if block.id != pick_block.id:
                    block.inflate()

    def fit_neighbors(self):
        for block in self._blocks.values():
            pick_block = self._blackboard.get("next_cube")
            if pick_block is None:
                return
            if not block.properties.get("in_stack"):
                if block.id != pick_block.id:
                    block.deflate()

    def print_block_info(self):
        """ debugging """
        for key, value in self._blocks.items():
            print("block: " + str(key))
            print(value.id)
            print(value.pose)
            print(value.color)
