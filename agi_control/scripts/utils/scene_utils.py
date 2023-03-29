#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""This class implements usefull functions that relate to the scene."""
import pdb
# sklearn
from sklearn.neighbors import KDTree
from sklearn.cluster import DBSCAN

# numpy
import numpy as np
import matplotlib.pyplot as plt

# rospy
import rospy
from rospy.rostime import Time

# movit
from moveit_commander import PlanningSceneInterface
from moveit_msgs.srv import GetPlanningScene

# Ros messages
from visualization_msgs.msg import Marker

#PyTrees
import py_trees

import matplotlib.pyplot as plt
from behaviors.arm_selection import Arm


class SceneUtils():

    def __init__(self):
        self._scene = PlanningSceneInterface()
        self._scene_srv = rospy.ServiceProxy("/get_planning_scene",
                                             GetPlanningScene)
        self._scene_srv.wait_for_service()
        self._blackboard = py_trees.blackboard.Blackboard()

    def get_scene_blocks(self):
        """This function returns the collision objects of the planning scene that have an id that starts with 'box'."""
        scene = self._scene_srv().scene.world.collision_objects
        blocks = [obj for obj in scene if obj.id.startswith("box")]
        return blocks

    def get_table_block(self):
        """This function returns the collision object of the planning scene that has an id that starts with 'table'."""
        scene = self._scene_srv().scene.world.collision_objects
        table = [obj for obj in scene if obj.id.startswith("table")]
        return table[0]

    def get_free_region(self):
        """This function finds n regions in an area that is not occupied by any collision object.

            The function takes in a list of collision objects that represent the blocks in the scene and a collision object that
            represents the table. It uses the dimensions of the table as bounds of the area to find the regions in. It then
            finds the regions that are not occupied by any collision object.
            This is done using a KDTree to find the closest collision object to each point in the area. If the closest collision
            object is farther than a threshold from the point, then the point is considered to be in a free region.
            The function then returns the n largest regions that are not occupied by any collision object as a list of
            points and the radius of the regions.
            
            Args:
                scene_blocks (list): A list of collision objects that represent the blocks in the scene
                table_block (CollisionObject): The collision object that represents the table
                num_regions (int): The number of regions to find
                
            Returns:
                list: A list of points that represent the center of the regions with the radius of the regions
            
            """
        # Get the table dimensions\
        table_block = self._blackboard.get('table_co')
        scene_cubes = self._blackboard.get('scene_cubes')
        table_dimensions = table_block.primitives[0].dimensions
        table_pose = table_block.primitive_poses[0]
        table_x_min, table_x_max = (table_pose.position.x -
                                    table_dimensions[0] / 2,
                                    table_pose.position.x +
                                    table_dimensions[0] / 2)
        table_y_min, table_y_max = (table_pose.position.y -
                                    table_dimensions[1] / 2,
                                    table_pose.position.y +
                                    table_dimensions[1] / 2)

        # Create a KDTree from the scene blocks
        cube_points = np.array(
            [[cube.pose.pose.position.x, cube.pose.pose.position.y]
             for cube in scene_cubes])
        cube_tree = KDTree(cube_points)

        # Find the closest block to each point in the area
        x = np.linspace(table_x_min, table_x_max, 50)
        y = np.linspace(table_y_min, table_y_max, 50)
        xx, yy = np.meshgrid(x, y)
        points = np.array([xx.flatten(), yy.flatten()]).T
        distances, indices = cube_tree.query(points, return_distance=True)
        # Find the points that are not occupied by any block
        free_points = points[(distances > 0.05).flatten()]
        # Remove points that are on the edge of the table
        thres = 0.15
        free_points = free_points[(free_points[:, 0] > table_x_min + thres)
                                  & (free_points[:, 0] < table_x_max - thres) &
                                  (free_points[:, 1] > table_y_min + thres) &
                                  (free_points[:, 1] < table_y_max - thres)]

        reachability = Arm(free_points)

        # Add z coordinate to free_points and set it to table height +  offset for placing the
        free_points = np.hstack((free_points, np.ones(
            (free_points.shape[0], 1)) * ((table_pose.position.z * 2) + 0.0375)))

        r_left = reachability.getScore3D(reachability.map3D_l, free_points)
        r_right = reachability.getScore3D(reachability.map3D_r, free_points)
        r_mean = (np.asarray(r_left) + np.asarray(r_right)) / 2
        r_norm = (r_mean - np.min(r_mean)) / (np.max(r_mean) - np.min(r_mean))
        max_reach = np.argmax(r_norm)

        # Plot the free points with reachability. The color of the points is
        # determined by the reachability. Green is good, blue is bad.
        # fig = plt.figure()
        # ax = fig.add_subplot(111)
        # # Rotate the plot so that the x axis is horizontal
        # ax.scatter(free_points[:, 0], free_points[:, 1], c=r_norm, s=50)

        # # plot point with highest reachability
        # ax.scatter(free_points[max_reach, 0],
        #            free_points[max_reach, 1],
        #            c='r',
        #            s=50)

        # plt.show()
        max_reach_point = free_points[max_reach, :]
        max_reach_point = max_reach_point.tolist()
        return max_reach_point

    @staticmethod
    def create_marker(position, radius):
        marker = Marker()
        marker.header.frame_id = "/base_footprint"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.scale.x = radius
        marker.scale.y = radius
        marker.scale.z = radius
        marker.color.a = .5
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        return marker
