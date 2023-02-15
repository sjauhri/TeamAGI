#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

# Import modules
import rospy
# import ros_numpy
import pcl
# https://github.com/strawlab/python-pcl
import numpy as np
import ctypes
import struct
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from random import randint
from std_msgs.msg import String
from sensor_msgs.msg import Image 



def callback(data):
    # gets pointcloud2 data from node 
    if not data.is_dense:
        rospy.logwarn('invalid points in Pointcloud!')
    # rospy.loginfo(rospy.get_caller_id() + ('image  width %s and height %s' % (data.width, data.height)))

    
    # for p in pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True):
    #     print " x : %f  y: %f  z: %f" %(p[0],p[1],p[2])
    pcl_data = ros_to_pcl(data)
    extracting_indices(pcl_data)
    #print(pcl)
    segment_pcl(pcl_data, data)

# link for clusters: https://github.com/jupidity/PCL-ROS-cluster-Segmentation/blob/master/src/segmentation.cpp
    


def pose_estimation():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('image_getter', anonymous=True)

    # http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html
    rospy.Subscriber('/xtion/depth_registered/points', PointCloud2, callback)
    # /xtion/depth_registered/camera_info
    # /xtion/depth_registered/image_raw
    # /xtion/depth_registered/points
    rospy.loginfo('started get image node') # %s and height %s' % (data.width, data.height))
    # spin() simply keeps python from exiting until this node is stopped

    
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        
        pub.publish(hello_str)
        rate.sleep()

    rospy.spin()

from pcl_handler import PCLHandler

if __name__ == '__main__':
    # pose_estimation()
    PCLHandler()
    
