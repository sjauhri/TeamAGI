#! /usr/bin/env python
# -*- coding: utf-8 -*-
'''
According to the subscribed '/cube_pose' topic, 
judge the reachability of the left and right hands, 
and then publish the '/armNum' topic (0=unreachable, 1=left, 2=right)

Note: map files in the map folder need to change permissions (chmod a+x file)
'''

import os
import numpy as np
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Int8




# def init():
#     x = -1 + 2*random.random()
#     y = -1 + 2*random.random()
#     z = -1 + 2*random.random()
#     pos = np.array(([x,y,z]))
#     # print(pos)
#     return pos

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

def callback(msg):
    pos = np.empty((3,1))
    pos[0] = msg.pose.position.x
    pos[1] = msg.pose.position.y
    pos[2] = msg.pose.position.z
    # rospy.loginfo(pos)
    
    map_left, map_right = reachMap()    
    # better reachability score(0~100) -> select arm
    score_l = score(map_left, pos)  # minmal distance and index
    score_r = score(map_right, pos)
    arm = selectArm(score_l,score_r)  # 0=unreachable, 1=left, 2=right
    rospy.loginfo("subscribe pose:")
    rospy.loginfo(msg.pose)
    try:
        armNum_publisher(arm)
    except rospy.ROSInterruptException:
        pass


def pose_subscriber():
    rospy.init_node('pose_subscriber','armNum_publisher', anonymous=True)
    rospy.Subscriber("/cube_pose", PoseStamped, callback)
    rospy.spin()
    
def armNum_publisher(arm):
    # 0=unreachable, 1=left, 2=right
    # rospy.init_node('armNum_publisher', anonymous=True)
    armNum_pub = rospy.Publisher('/armNum', Int8, queue_size=1)
    rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    arm_msg = Int8()
    arm_msg = arm
    armNum_pub.publish(arm_msg)
    rospy.loginfo("publish arm message: %d", arm_msg)
    rospy.loginfo("--------------------------------")
    rate.sleep()
     
def main():
    pose_subscriber()        
    
    
if __name__ == '__main__':
    main()