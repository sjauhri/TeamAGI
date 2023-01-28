#! /usr/bin/env python
# -*- coding: utf-8 -*-
'''
pulish topic of pose of cube, to test reachablility
'''

import rospy
from geometry_msgs.msg import Pose,PoseStamped
import random

def pose_publisher():
    rospy.init_node('pose_publisher', anonymous=True)
    cube_pose_pub = rospy.Publisher('/cube_pose', PoseStamped, queue_size=1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = -1 + 2*random.random()
        pose_msg.pose.position.y = -1 + 2*random.random()
        pose_msg.pose.position.z = -1 + 2*random.random()
        cube_pose_pub.publish(pose_msg)
        rospy.loginfo("publish cube-pose message: ")
        rospy.loginfo(pose_msg.pose)
        rospy.loginfo("--------------------------------")
        rate.sleep()
        
if __name__ == '__main__':
    try:
        pose_publisher()
    except rospy.ROSInterruptException:
        pass
