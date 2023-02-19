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
import pickle
import timeit


def init_map():
    # npy file -> array (reach map)
    # search the path of maps folder
    path_curr = os.getcwd()
    path_pre = os.path.abspath(os.path.dirname(path_curr))
    path_pre_pre = os.path.abspath(os.path.dirname(path_pre))
    if os.path.exists(os.path.join(path_curr, 'maps')):
        path_maps = path_curr
    elif os.path.exists(os.path.join(path_pre, 'maps')):
        path_maps = path_pre
    else:
        path_maps = path_pre_pre
    # load the maps
    path_map_left = os.path.join(path_maps, \
                                'maps/reach_map_6D_left_clipped.pkl')
    path_map_right = os.path.join(path_maps, \
                                'maps/reach_map_6D_right_clipped.pkl')
    map_left = pickle.load(open(path_map_left, "rb"))
    map_right = pickle.load(open(path_map_right, "rb"))

    return [map_left, map_right]


class Arm():

    def __init__(self, pose, map_l, map_r):
        self.pos = pose
        self.map_l = map_l
        self.map_r = map_r

    # def reachMap(self):
    #     # npy file -> array (reach map)
    #     start = timeit.default_timer()

    #     # search the path of maps folder
    #     path_curr = os.getcwd()
    #     path_pre = os.path.abspath(os.path.dirname(path_curr))
    #     path_pre_pre = os.path.abspath(os.path.dirname(path_pre))
    #     if os.path.exists(os.path.join(path_curr, 'maps')):
    #         path_maps = path_curr
    #     elif os.path.exists(os.path.join(path_pre, 'maps')):
    #         path_maps = path_pre
    #     else:
    #         path_maps = path_pre_pre
    #     # load the maps
    #     path_map_left = os.path.join(path_maps, \
    #                                 'maps/reach_map_6D_left_clipped.pkl')
    #     path_map_right = os.path.join(path_maps, \
    #                                 'maps/reach_map_6D_right_clipped.pkl')
    #     map_left = pickle.load(open(path_map_left, "rb"))
    #     map_right = pickle.load(open(path_map_right, "rb"))

    #     end = timeit.default_timer()
    #     print('Running time of load map: %s Seconds' % (end - start))

    #     return map_left, map_right

    def score(self, map, pos, left_right):
        # reach map,poses -> list of scores
        list_score = []
        for p in pos:

            # Voxelize the pose
            min_x, max_x, = (map[0, 0], map[-1, 0])
            min_y, max_y, = (map[0, 1], map[-1, 1])
            min_z, max_z, = (map[0, 2], map[-1, 2])
            min_roll, max_roll, = (map[0, 3], map[-1, 3])
            min_pitch, max_pitch, = (map[0, 4], map[-1, 4])
            min_yaw, max_yaw, = (map[0, 5], map[-1, 5])
            cartesian_res = 0.05
            angular_res = np.pi / 8
            # print("min_x, max_x:", min_x,"~", max_x)
            # print("min_y, max_y:", min_y,"~", max_y)
            # print("min_z, max_z:", min_z,"~", max_z)
            # print("min_roll, max_roll:", min_roll,"~", max_roll)
            # print("min_pitch, max_pitch:", min_pitch,"~", max_pitch)
            # print("min_yaw, max_yaw:",min_yaw,"~", max_yaw)
            # print(np.max(map[:,5]))

            x_bins = np.ceil((max_x - min_x) / cartesian_res)
            y_bins = np.ceil((max_y - min_y) / cartesian_res)
            z_bins = np.ceil((max_z - min_z) / cartesian_res)
            roll_bins = np.ceil((max_roll - min_roll) / angular_res)
            pitch_bins = np.ceil((max_pitch - min_pitch) / angular_res)
            yaw_bins = np.ceil((max_yaw - min_yaw) / angular_res)

            # Define the offset values for indexing the map
            x_ind_offset = y_bins * z_bins * roll_bins * pitch_bins * yaw_bins
            y_ind_offset = z_bins * roll_bins * pitch_bins * yaw_bins
            z_ind_offset = roll_bins * pitch_bins * yaw_bins
            roll_ind_offset = pitch_bins * yaw_bins
            pitch_ind_offset = yaw_bins
            yaw_ind_offset = 1

            # Convert the input pose to voxel coordinates
            x_idx = int(np.floor(p[0] / (cartesian_res / x_bins)))
            y_idx = int(np.floor(p[1] / (cartesian_res / y_bins)))
            z_idx = int(np.floor(p[2] / (cartesian_res / z_bins)))
            roll_idx = int(np.floor(
                (p[3] + np.pi) / (angular_res / roll_bins)))
            pitch_idx = int(
                np.floor((p[4] + np.pi) / (angular_res / pitch_bins)))
            yaw_idx = int(np.floor((p[5] + np.pi) / (angular_res / yaw_bins)))

            # Compute the index in the reachability map array
            map_idx = x_idx * x_ind_offset + y_idx * y_ind_offset + z_idx * z_ind_offset + roll_idx  \
            * roll_ind_offset + pitch_idx * pitch_ind_offset + yaw_idx * yaw_ind_offset

            list_score.append(map[int(map_idx), -1])
        #   print(map_idx)
        # print("-----------------------------------------")

        return list_score

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
        # better reachability score -> select arm
        # map_left, map_right = self.reachMap()
        map_left = self.map_l
        map_right = self.map_r
        list_arm = []

        start = timeit.default_timer()

        list_score_l = self.score(map_left, self.pos, "left")
        list_score_r = self.score(map_right, self.pos, "right")

        end = timeit.default_timer()
        print('Running time of get scores: %s Seconds' % (end - start))

        for i in range(len(list_score_l)):
            list_arm.append(self.selectArm(
                list_score_l[i],
                list_score_r[i]))  # unreachable="", left="left", right="right"
        print("score_l:", list_score_l)
        print("score_r", list_score_r)
        # print(arm)
        if len(list_arm) == 1:
            return list_arm[0]
        else:
            return list_arm


if __name__ == '__main__':
    # x = -0.48 + 0.95 * (np.random.random())  # x: -0.48 ~ 0.47
    # y = -0.32 + 0.64 * (np.random.random())  # y: -0.32 ~ 0.32
    # z = 0.33 + 1 * (np.random.random())  # z: 0.33 ~ 0.68
    # rx = (-1 + 2 * random.random()) * 3
    # ry = (-1 + 2 * np.random.random()) * 1.4
    # rz = (-1 + 2 * random.random()) * 3
    x, y, z, rx, ry, rz = (0.2, 0.2, 0.48, 0.1, 0.15, 0.21)
    pose1 = np.array(([[x, y, z, rx, ry, rz]]))
    pose2 = np.array(([[x, y, z, rx, ry, rz], [x, y, z, rx, ry, rz],
                       [x, y, z, rx, ry, rz]]))
    maps = init_map()

    print("shape of input poses:", pose1.shape)
    arm = Arm(pose1, maps[0], maps[1])
    print(arm.getArm())
