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

    def score(self, map, pos):
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

            x = np.argwhere(map[:,0]==round(pose[0],))
            y = np.argwhere(map[:,1]==pose[1])
            z = np.argwhere(map[:,2]==pose[2])
            print(x,y,z)

        print("x:",np.min(map[:,0]),"~",np.max(map[:,0]))
        print("y:",np.min(map[:,1]),"~",np.max(map[:,1]))
        print("z:",np.min(map[:,2]),"~",np.max(map[:,2]))
        print("pos:",pos.reshape(-1)[0:3])
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

        list_score_l = self.score(map_left,
                                  self.pos)  # minmal distance and index
        list_score_r = self.score(map_right, self.pos)

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
    x = -0.9 + 1.7 * (np.random.random())  # x: -0.92499995 ~ 0.875
    y = -0.3 + 1.4 * (np.random.random())  # y: -0.325 ~ 1.125
    z = 0.425 + 0.1 * (np.random.random())  # z: 0.425 ~ 0.52500004
    rx = (-1 + 2 * random.random()) * 3
    ry = (-1 + 2 * np.random.random()) * 1.4
    rz = (-1 + 2 * random.random()) * 3
    pose1 = np.array(([[x, y, z, rx, ry, rz]]))
    pose2 = np.array(([[x, y, z, rx, ry, rz], [x, y, z, rx, ry, rz],
                       [x, y, z, rx, ry, rz]]))

    print("shape of input poses:", pose1.shape)
    arm = Arm(pose1)
    print(arm.getArm())
