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
from scipy.spatial.transform import Rotation
import pdb


class Arm():
    # class-level variable to keep track of whether maps have been loaded or not
    _maps_loaded = False

    def __init__(self, pose):
        if pose.shape[1] == 7:
            pose_quat = pose[:,3:]
            pose_euler = Rotation.from_quat(pose_quat).as_euler('xyz', degrees=True)
            pose = np.hstack((pose[:,:3], pose_euler))
        self.pos = pose
        if not Arm._maps_loaded:  # if maps haven't been loaded yet, load them
            Arm.map6D_l, Arm.map6D_r, Arm.map3D_l, Arm.map3D_r = self.load_maps(
            )
            # set class-level variable to True to indicate that maps have been loaded
            Arm._maps_loaded = True

    def find_maps_folder(self):
        # Search in current directory and upper three levels
        for i in range(4):
            parent_dirs = os.path.join(*([os.pardir] * i) + [''])
            maps_path = os.path.join(os.getcwd(), parent_dirs,
                                     "6D3D_reach_maps")
            if os.path.exists(maps_path):
                return maps_path

        # Search in lower three levels
        for dirpath, dirnames, filenames in os.walk("."):
            if "6D3D_reach_maps" in dirnames:
                return os.path.join(dirpath, "6D3D_reach_maps")

        # If maps folder is not found, return path "/src/TeamAGI/agi_control/scripts/maps"
        # /home/hypatia/tiago_agi_ws/src/TeamAGI/agi_control/scripts/behaviors/arm_selection.py
        return os.path.join(os.getcwd(), "tiago_agi_ws", "src", "TeamAGI",
                            "agi_control", "scripts", "behaviors", "maps")

    def load_maps(self):
        maps_folder = self.find_maps_folder()
        map6D_l, map6D_r = None, None
        map3D_l, map3D_r = None, None
        for root, dirs, files in os.walk(maps_folder):
            if "score_map_left.h5" in files:
                with h5py.File(os.path.join(root, "score_map_left.h5"),
                               "r") as f:
                    map6D_l = np.array(f["map_l_s"])
            if "score_map_right.h5" in files:
                with h5py.File(os.path.join(root, "score_map_right.h5"),
                               "r") as f:
                    map6D_r = np.array(f["map_r_s"])
            if "filtered_3D_reach_map_gripper_left.npy" in files:
                map3D_l = np.load(
                    os.path.join(root,
                                 "filtered_3D_reach_map_gripper_left.npy"))
            if "filtered_3D_reach_map_gripper_right.npy" in files:
                map3D_r = np.load(
                    os.path.join(root,
                                 "filtered_3D_reach_map_gripper_right.npy"))

        return map6D_l, map6D_r, map3D_l, map3D_r

    def getScore6D(self, map, pos, left_right):
        # reach map,poses -> list of scores
        list_score = []
        for p in pos:
            if left_right == "l":
                min_y, max_y, = (-1.35, 0.6)
            else:
                min_y, max_y, = (-0.6, 1.35)
            min_x, max_x, = (-1.2, 1.2)
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
            score = map[np.floor(map_idx).astype(int)]
            list_score.append(score)

        return list_score

    def selectArm(self, score_l, score_r):
        if score_l == 0 and score_r == 0:
            # print("the inputed pose isn't in the reach map")
            return None
        elif score_l != 0 and score_r == 0:
            return "left"
        elif score_l == 0 and score_r != 0:
            return "right"
        elif score_l > score_r:
            # print("left arm")
            return "left"
        else:
            # print("right arm")
            return "right"

    # def getScore3D(self,map,pos):
    #     list_score = []
    #     for p in pos:
    #         # reach map,position -> score
    #         dist = np.empty((map.shape[0],1))
    #         for i,l in enumerate(map):
    #             dist[i] = np.sqrt(np.sum(np.square(l[:3]-p)))
    #         # which cell of 5cm boxel grid
    #         index = np.argmin(dist)
    #         score = map[index,3]
    #         list_score.append(score)
    #     return list_score

    def getScore3D(self, map, pos):
        dist = np.sqrt(
            np.sum(np.square(map[:, :3, np.newaxis] - pos.T), axis=1))
        index = np.argmin(dist, axis=0)
        list_score = map[index, 3].tolist()
        return list_score

    def getScore3D_from6DMap(self, map, pos, left_right):
        # reach map,poses -> list of scores
        list_score = []
        for p in pos:
            if left_right == "l":
                min_y, max_y, = (-1.35, 0.6)
            else:
                min_y, max_y, = (-0.6, 1.35)
            min_x, max_x, = (-1.2, 1.2)
            min_z, max_z, = (-0.35, 2.1)
            min_roll, max_roll, = (-np.pi, np.pi)
            min_pitch, max_pitch, = (-np.pi / 2, np.pi / 2)
            min_yaw, max_yaw, = (-np.pi, np.pi)

            p = np.append(p, [min_roll, min_pitch, min_yaw], axis=0)

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
            score_idx_ = np.floor(map_idx).astype(int)
            bins = int(roll_bins * pitch_bins * yaw_bins)
            score = np.mean(map[score_idx_:score_idx_ + bins])
            list_score.append(score)

        return list_score

    def getArm(self):
        # better reachability score -> select arm
        list_arm = []
        if self.pos.shape[1] == 6:
            list_score_l = self.getScore6D(Arm.map6D_l, self.pos, "l")
            list_score_r = self.getScore6D(Arm.map6D_r, self.pos, "r")
        elif self.pos.shape[1] == 3:
            list_score_l = self.getScore3D_from6DMap(Arm.map6D_l, self.pos,
                                                     "l")
            list_score_r = self.getScore3D_from6DMap(Arm.map6D_r, self.pos,
                                                     "l")
            # list_score_l = self.getScore3D(Arm.map6D_l, self.pos)
            # list_score_r = self.getScore3D(Arm.map6D_r, self.pos)

        else:
            raise ValueError(
                "Error: the inputed map shape should be (1,6),(n,6),(1,3) or (n,3)."
            )
        # print("score_l:",list_score_l)
        # print("score_r:",list_score_r)

        for i in range(len(list_score_l)):
            list_arm.append(self.selectArm(
                list_score_l[i],
                list_score_r[i]))  # unreachable="", left="left", right="right"

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
    position1 = np.array(([[x, y, z]]))
    position2 = np.array(([[x, y, z], [x, y, z], [x, y, z]]))

    print("6D reach: ------------------------------------")
    print("shape of input poses:", pose1.shape)
    start = timeit.default_timer()
    arm = Arm(pose1)
    print(arm.getArm())
    end = timeit.default_timer()
    print('Running time of get scores: %s Seconds' % (end - start))

    print("shape of input poses:", pose2.shape)
    start = timeit.default_timer()
    arm = Arm(pose2)
    print(arm.getArm())
    end = timeit.default_timer()
    print('Running time of get scores: %s Seconds' % (end - start))

    print("3D reach: ------------------------------------")
    print("shape of input positions:", position1.shape)
    start = timeit.default_timer()
    arm = Arm(position1)
    print(arm.getArm())
    end = timeit.default_timer()
    print('Running time of get scores: %s Seconds' % (end - start))

    print("shape of input positions:", position2.shape)
    start = timeit.default_timer()
    arm = Arm(position2)
    print(arm.getArm())
    end = timeit.default_timer()
    print('Running time of get scores: %s Seconds' % (end - start))
