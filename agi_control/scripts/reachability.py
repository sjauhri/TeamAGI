#! /usr/bin/env python
# -*- coding: utf-8 -*-
'''
According to the subscribed '/cube_pose' topic, 
judge the reachability of the left and right hands, 

Note: map files in the map folder need to change permissions (chmod a+x file)
'''

import os
import numpy as np
import random
import sys
import pdb

sys.path.append('..')
# from maps import f


class Arm():

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z
        self.pos = np.array(([self.x, self.y, self.z]))
        # print(self.pos)

    def reachMap(self):
        # npy file -> array (reach map)
        file_path = os.path.dirname(os.path.realpath(__file__))
        path_map_left = os.path.join(file_path, \
                                    '../maps/filtered_3D_reach_map_gripper_left_grasping_frame_torso_False_0.05_2022-08-29-19-04-04.npy')
        path_map_right = os.path.join(file_path, \
                                    '../maps/filtered_3D_reach_map_gripper_right_grasping_frame_torso_False_0.05_2022-08-29-19-04-04.npy')
        map_left = np.load(path_map_left)  #(x,y,z,score)
        map_right = np.load(path_map_right)
        return map_left, map_right

    def score(self, map, pos):
        # reach map,position -> score
        dist = np.empty((map.shape[0], 1))
        for i, l in enumerate(map):
            dist[i] = np.sqrt(np.sum(np.square(l[:3] - pos)))
        # which cell of 5cm boxel grid
        index = np.argmin(dist)
        score = map[index, 3]
        return score

    def selectArm(self, score_l, score_r):
        if score_l == 0 and score_r == 0:
            # print("the inputed pose isn't in the reach map")
            return ""
        elif score_l > score_r:
            # print("left arm")
            return "left"
        else:
            # print("right arm")
            return "right"

    def getArm(self):
        map_left, map_right = self.reachMap()
        # better reachability score(0~100) -> select arm
        score_l = self.score(map_left, self.pos)  # minmal distance and index
        score_r = self.score(map_right, self.pos)
        arm = self.selectArm(
            score_l, score_r)  # unreachable="", left="left", right="right"
        print("score_l:", score_l)
        print("score_r", score_r)
        print(arm)
        return arm


if __name__ == '__main__':
    x = -1 + 2 * random.random()
    y = -1 + 2 * np.random.random()
    z = -1 + 2 * random.random()
    arm = Arm(x, y, z)
    print(arm.getArm())
