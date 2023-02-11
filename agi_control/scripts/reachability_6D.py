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
        
    def __init__(self,x,y,z,rx,ry,rz):
        self.x = x
        self.y = y
        self.z = z
        self.rx = rx
        self.ry = ry
        self.rz = rz
        self.pos = np.array(([self.x,self.y,self.z,self.rx,self.ry,self.rz]))
        # print(self.pos)
    # def __init__(self):
    #     pass

    def reachMap(self):
        # npy file -> array (reach map)
        # search the path of maps folder
        start=timeit.default_timer()
        
        path_curr = os.getcwd()
        path_pre = os.path.abspath(os.path.dirname(path_curr))
        path_pre_pre = os.path.abspath(os.path.dirname(path_pre))        
        if os.path.exists(os.path.join(path_curr,'maps')):
            path_maps = path_curr
        elif os.path.exists(os.path.join(path_pre,'maps')):
            path_maps = path_pre
        else:
            path_maps = path_pre_pre
        # print(path_curr)
        # print(path_maps)   
        # path_map_left = os.path.join(path_maps, \
        #                             'maps/filtered_3D_reach_map_gripper_left_grasping_frame_torso_False_0.05_2022-08-29-19-04-04.npy')
        # path_map_right = os.path.join(path_maps, \
        #                             'maps/filtered_3D_reach_map_gripper_right_grasping_frame_torso_False_0.05_2022-08-29-19-04-04.npy')
        # map_left = np.load(path_map_left)    #(x,y,z,score)
        # map_right = np.load(path_map_right)
        path_map_left = os.path.join(path_maps, \
                                    'maps/filt_reach_map_gripper_left_grasping_frame_torso_False.pkl')
        path_map_right = os.path.join(path_maps, \
                                    'maps/filt_reach_map_gripper_right_grasping_frame_torso_False.pkl')
        map_left = pickle.load(open(path_map_left,"rb"))    
        map_right = pickle.load(open(path_map_right,"rb")) 
        # print(np.max(map_left, axis=0))
        # print(np.min(map_left, axis=0))
        # print(map_right.shape)
        # print(map_right[-10:,:])
        
        end=timeit.default_timer()
        print('Running time of load map: %s Seconds'%(end-start))
        
        return map_left, map_right

    def score(self,map,pos):              
        # reach map,position -> score
        # find the nearst cell in the reach map
        squ_dist = np.sum((map[:,:3]-pos[:3])**2,axis=1)
        inds_min_dist = np.argwhere(squ_dist<=squ_dist[np.argmin(squ_dist)]).reshape(-1)
        cell = map[inds_min_dist,:]
        
        # in the cell find the similarst rotation
        squ_rota = np.sum((cell[:,3:6]-pos[3:6])**2,axis=1)
        inds_min_rota = np.argwhere(squ_rota<=squ_rota[np.argmin(squ_rota)]).reshape(-1)
        
        score = map[inds_min_dist[inds_min_rota],7]
        return score



    def selectArm(self,score_l, score_r): 
        if score_l==0 and score_r==0:
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
        start=timeit.default_timer()
        
        score_l = self.score(map_left, self.pos)  # minmal distance and index
        score_r = self.score(map_right, self.pos)
              
        end=timeit.default_timer()
        print('Running time of get scores: %s Seconds'%(end-start))
        
        arm = self.selectArm(score_l, score_r)  # unreachable="", left="left", right="right"
        print("score_l:",score_l)
        print("score_r",score_r)
        # print(arm)
  
        return arm


     
    
if __name__ == '__main__':
    x = -1 + 2*random.random()
    y = -1 + 2*np.random.random()
    z = -1 + 2*random.random()
    rx = (-1 + 2*random.random())*3
    ry = (-1 + 2*np.random.random())*1.4
    rz = (-1 + 2*random.random())*3
    
    arm = Arm(x,y,z,rx,ry,rz)    
    print(arm.getArm()) 
    

    
    
    
    
    