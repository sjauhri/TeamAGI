#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""This file implements a simple stacking controller for the AGI robot."""


import rospy
from geometry_msgs.msg import PoseStamped
from moveit_commander import PlanningSceneInterface
from moveit_msgs.srv import GetPlanningScene

from tiago_dual_pick_place.srv import PickPlaceObject
from tiago_dual_pick_place.srv import PickPlaceAutoObject


class SimpleStacking(object):
    def __init__(self, number_of_blocks):
        self._scene = PlanningSceneInterface()
        rospy.loginfo("Connecting to /get_planning_scene service")
        self.scene_srv = rospy.ServiceProxy("/get_planning_scene", GetPlanningScene)
        self.scene_srv.wait_for_service()
        rospy.loginfo("Connected.")
        rospy.sleep(1)
        rospy.init_node("simple_stacking")
        self._number_of_blocks = number_of_blocks
        self._stack_location_pub = rospy.Publisher(
            "/place/pose",
            PoseStamped,
            queue_size=1,
            headers={"frame_id": "base_footprint"},
        )
        self._block_stack = []
        self._table_location = self._scene.get_object_poses(["table"])
        self._table_dimension = (
            self._scene.get_objects(["table"])["table"].primitives[0].dimensions
        )
        self._block_dimension = (
            self._scene.get_objects(["box0"])["box0"].primitives[0].dimensions
        )

    def start(self):
        """Start the stacking controller."""
        rospy.loginfo("Starting simple stacking controller")
        available_blocks = self.get_stacking_blocks()
        rospy.loginfo("Available blocks: " + str(available_blocks))
        for block in available_blocks:
            # Pick and place the block
            rospy.loginfo("Picking blocks")
            self.pick_block("left", block)
            rospy.sleep(1)
            rospy.loginfo("Placing blocks")
            place_location = self.next_stack_location()
            rospy.loginfo("Place location: " + str(place_location))
            self.place_block(block, place_location)
            self._block_stack.append(block)

    def get_stacking_blocks(self):
        """Return the list of available blocks in the scene.

        The blocks name start with "box" """

        all_blocks = [
            block
            for block in self._scene.get_known_object_names()
            if block.startswith("box")
        ]
        i = 0

        stacking_blocks = []
        while i < self._number_of_blocks:
            stacking_blocks.append(all_blocks[i])
            i += 1
        return stacking_blocks

    def pick_block(self, arm_name, block_name):
        """Pick a block from the table."""
        rospy.loginfo("Picking block: " + block_name)
        rospy.wait_for_service("/pick_object")
        pick_object = rospy.ServiceProxy("/pick_object", PickPlaceObject)
        pick_object(arm_name, block_name)

    def place_block(self, block_name, location):
        """Place a block on the table."""
        rospy.loginfo("Placing block: " + block_name)
        rospy.wait_for_service("/place_object")
        place_object = rospy.ServiceProxy("/place_object", PickPlaceAutoObject)
        place_object(block_name)
        self._stack_location_pub.publish(location)

    def next_stack_location(self):
        """Return the next stack location."""
        if len(self._block_stack) == 0:
            # First block
            location = PoseStamped()
            location.header.frame_id = "base_footprint"
            location.pose.position.x = self._table_location["table"].position.x
            location.pose.position.y = self._table_location["table"].position.y
            location.pose.position.z = (
                self._table_location["table"].position.z + self._table_dimension[2]
            )
            location.pose.orientation.w = 1.0
            return location
        else:
            last_block = self._block_stack[-1]
            print(last_block)
            last_block_location = self._scene.get_object_poses([last_block])

            location = PoseStamped()
            location.header.frame_id = "base_footprint"
            location.pose.position.x = last_block_location["box1"].position.x
            location.pose.position.y = last_block_location["box1"].position.y
            location.pose.position.z = (
                last_block_location["box1"].position.z + self._block_dimension[2]
            )
            location.pose.orientation.w = 1.0

            return location


if __name__ == "__main__":
    controller = SimpleStacking(4)
    controller.start()
