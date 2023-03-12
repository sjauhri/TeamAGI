#! /usr/bin/env python
# -*- coding: utf-8 -*-

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from moveit_msgs.msg import Grasp
from moveit_commander import PlanningSceneInterface
import rospy

from utils.robot_utils import get_arm


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
        rospy.sleep(0.5)
        self._collision_object = self._scene.get_objects([self._id])[self._id]
        self._properties = {}
        self._confidence = confidence
        self._left_right = get_arm(self)

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

    def add_property(self, name, value):
        self._properties[name] = value

    def grasps(self):
        grasps = []

        # Define the first grasp (top grasp)
        grasp1 = Grasp()
        grasp1.id = "grasp_1"
        grasp1.grasp_pose.header.frame_id = "base_footprint"
        grasp1.grasp_pose.pose.position.x = 0.0
        grasp1.grasp_pose.pose.position.y = 0.0
        grasp1.grasp_pose.pose.position.z = 0.5
        grasp1.grasp_pose.pose.orientation = self.orientation()

        grasps.append(grasp1)

        # Define the second grasp (front grasp)
        grasp2 = Grasp()
        grasp2.id = "grasp_2"
        grasp2.grasp_pose.header.frame_id = "base_footprint"
        grasp2.grasp_pose.pose.position.x = 0.0
        grasp2.grasp_pose.pose.position.y = 0.5
        grasp2.grasp_pose.pose.position.z = 0.25
        grasp1.grasp_pose.pose.orientation = self.orientation()

        grasps.append(grasp2)

        # Define the third grasp (right grasp)
        grasp3 = Grasp()
        grasp3.id = "grasp_3"
        grasp3.grasp_pose.header.frame_id = "base_footprint"
        grasp3.grasp_pose.pose.position.x = 0.5
        grasp3.grasp_pose.pose.position.y = 0.0
        grasp3.grasp_pose.pose.position.z = 0.25
        grasp1.grasp_pose.pose.orientation = self.orientation()

        grasps.append(grasp3)

        # Define the fourth grasp (back grasp)
        grasp4 = Grasp()
        grasp4.id = "grasp_4"
        grasp4.grasp_pose.header.frame_id = "base_footprint"
        grasp4.grasp_pose.pose.position.x = 0.0
        grasp4.grasp_pose.pose.position.y = -0.5
        grasp4.grasp_pose.pose.position.z = 0.25
        grasp1.grasp_pose.pose.orientation = self.orientation()

        grasps.append(grasp4)

        # Define the fifth grasp (left grasp)
        grasp5 = Grasp()
        grasp5.id = "grasp_5"
        grasp5.grasp_pose.header.frame_id = "base_footprint"
        grasp5.grasp_pose.pose.position.x = -0.5
        grasp5.grasp_pose.pose.position.y = 0.0
        grasp5.grasp_pose.pose.position.z = 0.25
        grasp1.grasp_pose.pose.orientation = self.orientation()

        grasps.append(grasp5)

        return grasps

    def update(self):
        try:
            self._scene.remove_world_object(self._id)
            self._scene.add_box(self._id, self._pose, (0.045, 0.045, 0.045))
            rospy.sleep(0.5)
            self._collision_object = self._scene.get_objects([self._id
                                                              ])[self._id]
        except:
            pass


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
        self._blocks = {}

    def add_block(self, id, pose, confidence, color=""):
        """This function adds a block to the block manager.
        Args:
            id (str): The id of the block
            pose (PoseStamped): The pose of the block
        """
        self._blocks[id] = Block(id, pose, confidence, color)

    def update_block(self, id, pose, confidence):
        """This function updates the pose of a block.
        Args:
            id (str): The id of the block
            pose (PoseStamped): The new pose of the block
        """
        block = self._blocks[id]
        block._pose = pose
        block._confidence = confidence
        block.update()

    def identify_block(self, pose, threshold=0.08, color=""):
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

    def manage(self, pose, confidence, color):
        """This function manages the blocks in the scene.
        Args:
            pose (PoseStamped): The pose of the block
        """
        id = self.identify_block(pose, color=color)
        if id is None:
            id = "box" + str(len(self._blocks))
            self.add_block(id, pose, confidence, color)
        else:
            self.update_block(id, pose, confidence)

        return id

    def print_block_info(self):
        """ debugging """
        for key, value in self._blocks.items():
            print("block: " + str(key))
            print(value.id)
            print(value.pose)
            print(value.color)
