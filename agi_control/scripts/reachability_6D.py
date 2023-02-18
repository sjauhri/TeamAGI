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


class Arm():

    def __init__(self, pose):
        self.pos = pose
        

    def reachMap(self):
        # npy file -> array (reach map)
        start = timeit.default_timer()

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

        end = timeit.default_timer()
        print('Running time of load map: %s Seconds' % (end - start))

        return map_left, map_right

    def score(self, map, pos, left_right):
        # reach map,poses -> list of scores
        list_score = []
        for p in pos:
            # find the nearst cell in the reach map
            pose = p.reshape(-1)
            squ_dist = np.sum((map[:, :3] - pose[:3])**2, axis=1)
            inds_min_dist = np.argwhere(
                squ_dist <= squ_dist[np.argmin(squ_dist)]).reshape(-1)
            cell = map[inds_min_dist, :]

            # in the cell find the similarst rotation
            squ_rota = np.sum((cell[:, 3:6] - pose[3:6])**2, axis=1)
            inds_min_rota = np.argwhere(
                squ_rota <= squ_rota[np.argmin(squ_rota)]).reshape(-1)

            list_score.append(map[inds_min_dist[inds_min_rota], 7])

            # find the input poses in the map
            # x = self.decimal_2(int(((p[0] - (map[0,0])) / 0.05))*0.05 -map[0,0])
            # if left_right == "left":
            #     y = self.decimal_2(int(((p[1] - (map[0,1])) / 0.05))*0.05 -map[0,1])
            # else:
            #     y = self.decimal_2(int(((p[1] - (map[0,2])) / 0.05))*0.05 + map[0,2])
            # z = self.decimal_2(int(((p[2] - (0.33)) / 0.05))*0.05 + 0.33)

            # dis = 0.05
            # p_r = np.round(p,1)
            # ind_x = np.argwhere(map[:,0]==p_r[0])[:,0]
            # ind_y = np.argwhere(map[ind_x,1]==p_r[1])[:,0]
            # ind_z = np.argwhere(map[ind_x,:][ind_y,2]==p_r[2])[:,0]
            # ind_pos = ind_x[ind_y][ind_z]
            # print(map.shape)
            # print(ind_pos.shape)
            # rad = 3.14/8
            # ind_rx = np.argwhere(map_pos[:,3]-p[3]<rad)[:,0]
            # ind_ry = np.argwhere(map_pos[ind_rx,4]-p[4]<rad)[:,0]
            # ind_rz = np.argwhere(map_pos[ind_rx[ind_ry],5]-p[5]<rad)[:,0]
            # map_rot = map_pos[ind_rx[ind_ry][ind_rz]]  
            
            # Define size of each voxel                
            min_x, max_x,  = (map[0,0],map[-1,0])
            min_y, max_y,  = (map[0,1],map[-1,1])
            min_z, max_z,  = (map[0,2],map[-1,2])
            min_roll, max_roll,  = (map[0,3],map[-1,3])
            min_pitch, max_pitch,  = (map[0,4],map[-1,4])
            min_yaw, max_yaw,  = (map[0,5],map[-1,5])
            xyz_bins = 0.05
            rpy_bins = 3.14/8

            x_size = (max_x - min_x) / xyz_bins
            y_size = (max_y - min_y) / xyz_bins
            z_size = (max_z - min_z) / xyz_bins
            roll_size = (max_roll - min_roll) / rpy_bins
            pitch_size = (max_pitch - min_pitch) / rpy_bins
            yaw_size = (max_yaw - min_yaw) / rpy_bins
            
            x_idx = int((p[0] - min_x) / x_size)
            y_idx = int((p[1] - min_y) / y_size)
            z_idx = int((p[2] - min_z) / z_size)
            roll_idx = int((p[3] - min_roll) / roll_size)
            pitch_idx = int((p[4] - min_pitch) / pitch_size)
            yaw_idx = int((p[5] - min_yaw) / yaw_size)
            print(x_idx,roll_idx)


        print("-----------------------------------------")

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
        map_left, map_right = self.reachMap()
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
    x,y,z,rx,ry,rz = (0.1,0.1,0.48, 0.1,0.15,0.21)
    pose1 = np.array(([[x, y, z, rx, ry, rz]]))
    pose2 = np.array(([[x, y, z, rx, ry, rz], [x, y, z, rx, ry, rz],
                       [x, y, z, rx, ry, rz]]))

    print("shape of input poses:", pose1.shape)
    arm = Arm(pose1)
    print(arm.getArm())
