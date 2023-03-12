#! /usr/bin/env python
# -*- coding: utf-8 -*-
'''
According to the inputed '/cube_pose' topic, 
judge the reachability of the left and right hands, 

Note: map files in the map folder may need to change permissions (chmod a+x file)
'''

import os
import numpy as np
# import pickle
import cPickle as pickle
import timeit
import h5py


class Arm():
    # class-level variable to keep track of whether maps have been loaded or not
    _maps_loaded = False  

    def __init__(self, pose):
        self.pos = pose
        if not Arm._maps_loaded:  # if maps haven't been loaded yet, load them
            Arm.map_l, Arm.map_r = self.load_maps()
            # set class-level variable to True to indicate that maps have been loaded
            Arm._maps_loaded = True  

    def find_maps_folder(self):
        # Search in current directory and upper three levels
        for i in range(4):
            parent_dirs = os.path.join(*([os.pardir] * i) + [''])
            maps_path = os.path.join(os.getcwd(), parent_dirs, "maps")
            if os.path.exists(maps_path):
                return maps_path

        # Search in lower three levels
        for dirpath, dirnames, filenames in os.walk("."):
            if "maps" in dirnames:
                return os.path.join(dirpath, "maps")

        # If maps folder is not found, return path "/src/TeamAGI/agi_control/scripts/maps"
        return os.path.join(os.getcwd(), "src", "TeamAGI", "agi_control",
                            "scripts", "maps")

    def load_maps(self):
        maps_folder = self.find_maps_folder()
        map_l, map_r = None, None
        for root, dirs, files in os.walk(maps_folder):
            if "score_map_left.h5" in files:
                with h5py.File(os.path.join(root, "score_map_left.h5"),
                               "r") as f:
                    map_l = np.array(f["map_l_s"])
            if "score_map_right.h5" in files:
                with h5py.File(os.path.join(root, "score_map_right.h5"),
                               "r") as f:
                    map_r = np.array(f["map_r_s"])
        return map_l, map_r

    def getScore(self, map, pos, left_right):
        # reach map,poses -> list of scores
        list_score = []
        for p in pos:
            if left_right == "l":
                min_x, max_x, = (-1.2, 1.2)
                min_y, max_y, = (-1.35, 0.6)
                min_z, max_z, = (-0.35, 2.1)
                min_roll, max_roll, = (-np.pi, np.pi)
                min_pitch, max_pitch, = (-np.pi / 2, np.pi / 2)
                min_yaw, max_yaw, = (-np.pi, np.pi)
            else:
                min_x, max_x, = (-1.2, 1.2)
                min_y, max_y, = (-0.6, 1.35)
                min_z, max_z, = (-0.35, 2.1)
                min_roll, max_roll, = (-np.pi, np.pi)
                min_pitch, max_pitch, = (-np.pi / 2, np.pi / 2)
                min_yaw, max_yaw, = (-np.pi, np.pi)

            cartesian_res = 0.05
            angular_res = np.pi / 8

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
            x_idx = int(np.floor((p[0] - min_x) / cartesian_res))
            y_idx = int(np.floor((p[1] - min_y) / cartesian_res))
            z_idx = int(np.floor((p[2] - min_z) / cartesian_res))
            roll_idx = int(np.floor((p[3] - min_roll) / angular_res))
            pitch_idx = int(np.floor((p[4] - min_pitch) / angular_res))
            yaw_idx = np.floor((p[5] - min_yaw) / angular_res).astype(int)

            # Compute the index in the reachability map array
            map_idx = x_idx * x_ind_offset + y_idx * y_ind_offset + z_idx * z_ind_offset + roll_idx  \
            * roll_ind_offset + pitch_idx * pitch_ind_offset + yaw_idx * yaw_ind_offset
            # Use modulo to avoid out of bounds index
            if map_idx > len(map):
                map_idx = map_idx % len(map)

            # Get the score from the score map array
            list_score.append(map[np.floor(map_idx).astype(int)])

        return list_score

    def selectArm(self, score_l, score_r):
        if score_l == 0 and score_r == 0:
            # print("the inputed pose isn't in the reach map")
            return "left"
        elif score_l > score_r:
            # print("left arm")
            return "left"
        else:
            # print("right arm")
            return "right"

    def getArm(self):
        # better reachability score -> select arm
        list_arm = []

        # start = timeit.default_timer()

        list_score_l = self.getScore(Arm.map_l, self.pos, "l")
        list_score_r = self.getScore(Arm.map_r, self.pos, "r")

        # end = timeit.default_timer()
        # print('Running time of get scores: %s Seconds' % (end - start))

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
    # maps = load_maps()

    print("shape of input poses:", pose1.shape)
    start = timeit.default_timer()
    arm = Arm(pose1)
    print(arm.getArm())
    end = timeit.default_timer()
    print('Running time of get scores: %s Seconds' % (end - start))
    
    print("shape of input poses:", pose1.shape)
    start = timeit.default_timer()
    arm = Arm(pose1)
    print(arm.getArm())
    end = timeit.default_timer()
    print('Running time of get scores: %s Seconds' % (end - start))
