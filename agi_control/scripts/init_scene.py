#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

#numpy
import numpy as np

# ros
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray
# movit
from moveit_commander import PlanningSceneInterface
from moveit_msgs.srv import GetPlanningScene

from utils.scene_utils import SceneUtils

# object presets

table = {"size": [0.60, 0.75, 0.45]}
box = {"size": [0.045, 0.045, 0.045], "mass": 0.66}


class InitScene:

    def __init__(self):
        self._scene = PlanningSceneInterface()
        rospy.loginfo("Connecting to /get_planning_scene service")
        self.scene_srv = rospy.ServiceProxy("/get_planning_scene",
                                            GetPlanningScene)
        self.scene_srv.wait_for_service()
        rospy.loginfo("Connected.")
        rospy.sleep(1)

    def add_box(self, name, pose, size):
        """Add a box to the planning scene"""
        rospy.loginfo("Adding {} to the planning scene".format(name))
        self._scene.add_box(name, pose, size)

    def create_table(self):
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base_footprint"
        table_pose.pose.position.x = 0.5
        table_pose.pose.position.y = 0
        table_pose.pose.position.z = table["size"][2] / 2
        table_pose.pose.orientation.w = 1.0
        self.add_box("table", table_pose, table["size"])

    def update_scene(self):
        su = SceneUtils()
        cube_poses = rospy.wait_for_message('/cube_poses', PoseArray)

        for i, pose in enumerate(cube_poses.poses):
            pose_stamp = PoseStamped()
            pose_stamp.header.frame_id = "base_footprint"
            pose_stamp.pose = pose
            self.add_box("box{}".format(i), pose_stamp, box["size"])

        boxes_obj = su.get_scene_blocks()
        table_obj = su.get_table_block()

        free_region = su.get_free_region(boxes_obj, table_obj)
        markers = MarkerArray()
        for region in free_region:
            loc = np.append(region[0], table["size"][2] + 0.01)
            rad = region[1]
            rospy.loginfo("Creating marker at {} with radius {}".format(
                loc, rad))
            markers.markers.append(su.create_marker(loc, rad))

        pub_markers = rospy.Publisher('/free_region',
                                      MarkerArray,
                                      queue_size=10)
        pub_markers.publish(markers)
        rospy.sleep(0.1)

    def clean_scene(self):
        """Clean the scene from all objects"""
        rospy.loginfo("Cleaning scene")
        for name in self._scene.get_known_object_names():
            self._scene.remove_world_object(name)


if __name__ == "__main__":
    rospy.init_node("init_scene")
    rate = rospy.Rate(10)
    init_scene = InitScene()
    init_scene.clean_scene()
    # init_scene.create_default_scene()
    init_scene.create_table()
    while not rospy.is_shutdown():
        init_scene.update_scene()

    rospy.spin()
