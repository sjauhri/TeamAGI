#!/usr/bin/env python

# # Import modules
# import rospy
# # import ros_numpy
# import pcl
# # https://github.com/strawlab/python-pcl
# import numpy as np
# import ctypes
# import struct
# import sensor_msgs.point_cloud2 as pc2

# from sensor_msgs.msg import PointCloud2, PointField
# from std_msgs.msg import Header
# from random import randint
# from std_msgs.msg import String
# from sensor_msgs.msg import Image 
import rospy
from pcl_handler import PCLHandler
from pcl_service import PCLService
import sys 
from agi_vision.srv import PerceptionSRV
from geometry_msgs.msg import PoseStamped


if __name__ == '__main__':
    rospy.wait_for_service('PerceptionService')
    perc_srv = rospy.ServiceProxy('PerceptionService', PerceptionSRV)
    try:
        loc = PoseStamped()
        # perc_res = perc_srv(n_stacked=int(3), location_stack=loc)
        perc_res = perc_srv(int(0), None)
        print(perc_res)
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))
