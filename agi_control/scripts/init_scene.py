#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy



# ros
from geometry_msgs.msg import PoseStamped

# movit
from moveit_commander import PlanningSceneInterface
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.srv import GetPlanningSceneRequest
from moveit_msgs.srv import GetPlanningSceneResponse

# object presets

table = {"size": [0.60, 0.75, 0.45]}
box = {"size": [0.045, 0.045, 0.045], "mass": 0.66}


class InitScene(object):

    def __init__(self):
        self._scene = PlanningSceneInterface()
        rospy.loginfo("Connecting to /get_planning_scene service")
        self.scene_srv = rospy.ServiceProxy(
            '/get_planning_scene', GetPlanningScene)
        self.scene_srv.wait_for_service()
        rospy.loginfo("Connected.")
        rospy.sleep(1)

    def add_box(self, object_name, pose, size):
        """Add a box to the planning scene"""
        rospy.loginfo("Adding new '"+str(object_name)+"' object")
        self._scene.add_box(object_name, pose, size)

    def create_default_scene(self):
        """Create a default scene with a table and a box"""
        table_pose = PoseStamped()
        table_pose.header.frame_id = "base_footprint"
        table_pose.pose.position.x = table["size"][0]
        table_pose.pose.position.y = 0
        table_pose.pose.position.z = table["size"][2]/2
        table_pose.pose.orientation.w = 1.0

        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_footprint"
        box_pose.pose.position.x = table_pose.pose.position.x + box["size"][0]/2 - 0.15
        box_pose.pose.position.y = table["size"][1]/4
        box_pose.pose.position.z = table["size"][2] + box["size"][2]/2 + 0.01
        box_pose.pose.orientation.w = 1.0

        rospy.loginfo("Adding default scene")
        #self.add_box("table", table_pose, table["size"])
        self.add_box("box1", box_pose, box["size"])

    def clean_scene(self):
        """Clean the scene from all objects"""
        rospy.loginfo("Cleaning scene")
        for name in self._scene.get_known_object_names():
            self._scene.remove_world_object(name)

if __name__ == "__main__":
    rospy.init_node("init_scene")
    init_scene = InitScene()
    init_scene.clean_scene()
    init_scene.create_default_scene()
    rospy.spin()