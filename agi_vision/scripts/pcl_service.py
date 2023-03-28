# Import modules
import rospy
# import ros_numpy
import tf2_ros
import tf2_py as tf2

import pcl
# https://github.com/strawlab/python-pcl
import numpy as np
import ctypes
import struct
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, PoseArray
from random import randint
from std_msgs.msg import String
from sensor_msgs.msg import Image
# http://docs.ros.org/en/noetic/api/shape_msgs/html/msg/Plane.html
from shape_msgs.msg import Plane
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from utils import k_means_clustering, XYZRGB_to_XYZ, find_edge, get_pose
# https://answers.ros.org/question/59725/publishing-to-a-topic-via-subscriber-callback-function/
import cluster_utils
import pdb
from pdb import set_trace as bp

from agi_vision.msg import PerceptionMSG
from cluster_handler import ClusterHandler
from pcl_handler import PCLHandler
from agi_vision.srv import PerceptionSRV
from datetime import datetime
import time


class PCLService:
    def __init__(self, n_cubes=5):
        print("n_cubes:", n_cubes)

        self.n_clusters = n_cubes
        self.node = rospy.init_node('pcl_service', anonymous=True)


        self.node_service = rospy.init_node('pcl_service', anonymous=True)

        # rospy.Service(name, service_class, handler, buff_size=65536)
        self.service = rospy.Service('PerceptionService', PerceptionSRV, self.request_handler)


        self.pcl_handler = PCLHandler(self.n_clusters)

        # publisher for perceptionMSG
        # self.pub_perception = rospy.Publisher('PerceptionMSG',
        #                                        PerceptionMSG,
        #                                        queue_size=10)


        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(1))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.spin()

    def request_handler(self, req):
        # TODO. other way of implementing it
        # let subscriber pubisher rolling 
        # store it in class attribute, let service return latest stored perceptin msg 
        print("working on request")
        print(req)
        n_stacked = req.n_stacked
        loc_stack = req.location_stack


        # loc = req.location_stack
        start_time = time.time() 
        time.sleep(1)
        # data = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2, timeout=10)
        data = rospy.wait_for_message('/xtion/depth_registered/points', PointCloud2, timeout=10)
        stamp = data.header.stamp

        cube_poses, color_names, confidences = self.pcl_handler(data, int(n_stacked), loc_stack)
        print("perceived")
        perc_msg = self.generate_msg(cube_poses, color_names, confidences, stamp)

        stop_time = time.time()  
        elapsed_time = stop_time - start_time
        print('elapsed time ', elapsed_time)
        return perc_msg


    def generate_msg(self, cube_poses, color_names, confidences, stamp=None):
        perc_msg = PerceptionMSG()
        perc_msg.size = len(color_names)

        if stamp is None:
            stamp = rospy.Time.now()
        perc_msg.data_stamp = stamp

        cube_poses.header.frame_id = "base_footprint"
        cube_poses.header.stamp = rospy.Time.now()
        perc_msg.pose_array = cube_poses

        perc_msg.color_names = color_names
        perc_msg.confidence = confidences
        return perc_msg
