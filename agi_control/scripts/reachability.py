#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import numpy as np
import random

def init():
    x = -1 + 2*random.random()
    y = -1 + 2*random.random()
    z = -1 + 2*random.random()
    pos = np.array(([x,y,z]))
    # print(pos)
    return pos

def reachMap():
    # npy file -> array (reach map)
    path_curr = os.getcwd()
    path_map_left = os.path.join(os.path.dirname(path_curr), \
                                 'maps/filtered_3D_reach_map_gripper_left_grasping_frame_torso_False_0.05_2022-08-29-19-04-04.npy')
    path_map_right = os.path.join(os.path.dirname(path_curr), \
                                 'maps/filtered_3D_reach_map_gripper_right_grasping_frame_torso_False_0.05_2022-08-29-19-04-04.npy')
    map_left = np.load(path_map_left)    #(x,y,z,score)
    map_right = np.load(path_map_right)
    return map_left, map_right

def score(map,pos):
    # reach map,position -> score
    dist = np.empty((map.shape[0],1))
    for i,l in enumerate(map):
        dist[i] = np.sqrt(np.sum(np.square(l[:3]-pos))) 
    # which cell of 5cm boxel grid
    index = np.argmin(dist)
    score = map[index,3]
    return score

def selectArm(score_l, score_r): 
    if score_l==0 and score_r==0:
        # print("the inputed pose isn't in the reach map")
        return 0   
    elif score_l > score_r:
        # print("left arm")
        return 1
    else:
        # print("right arm")
        return 2
        
def main():
    pos = init()  # position of input pose, array, (3,0)
    map_left, map_right = reachMap()
    
    # better reachability score(0~100) -> select arm
    score_l = score(map_left, pos)  # minmal distance and index
    score_r = score(map_right, pos)
    arm = selectArm(score_l,score_r)  # 0=unreachable, 1=left, 2=right
    print("input random position:", pos)
    print("score_left=",score_l)
    print("score_right=",score_r)
    print("arm=",arm,"(0=unreachable, 1=left, 2=right)")
    
        
    

    
if __name__ == '__main__':
    main()