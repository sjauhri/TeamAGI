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
class PCLHandler:
    def __init__(self):
        self.node = rospy.init_node('pcl_handling', anonymous=True)

        self.sub = rospy.Subscriber('/xtion/depth_registered/points', PointCloud2, self.callback)
        self.pub_pcl_filter = rospy.Publisher('Pointcloud2_filtered', PointCloud2, queue_size=10)
        self.pub_pcl_planes = rospy.Publisher('Pointcloud2_cube_planes', PointCloud2, queue_size=10)
        self.pub_pcl_edges = rospy.Publisher('Edge_points', PointCloud2, queue_size=10)


        self.pub1_edge = rospy.Publisher('Edge_point1', PointCloud2, queue_size=10)
        self.pub2_edge = rospy.Publisher('Edge_point2', PointCloud2, queue_size=10)
        self.pub3_edge = rospy.Publisher('Edge_point3', PointCloud2, queue_size=10)
        self.pub4_edge = rospy.Publisher('Edge_point4', PointCloud2, queue_size=10)
        self.pub5_edge = rospy.Publisher('Edge_point5', PointCloud2, queue_size=10)
        self.pub6_edge = rospy.Publisher('Edge_point6', PointCloud2, queue_size=10)
        self.pub7_edge = rospy.Publisher('Edge_point7', PointCloud2, queue_size=10)
        self.pub8_edge = rospy.Publisher('Edge_point8', PointCloud2, queue_size=10)

        self.pub3_pose = rospy.Publisher('cube_poses', PoseArray, queue_size=10)
        # self.pub_plane = rospy.Publisher('Plane_rk', Plane, queue_size=10)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(1))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.spin()

     
    def callback(self, data):
        stamp = data.header.stamp
        # gets pointcloud2 data from node 
        if not data.is_dense:
            rospy.logwarn('invalid points in Pointcloud!')
        # rospy.loginfo(rospy.get_caller_id() + ('image  width %s and height %s' % (data.width, data.height)))
        data=self.transform_to_base(data, stamp)  
        # for p in pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True):
        #     print " x : %f  y: %f  z: %f" %(p[0],p[1],p[2])

        n_clusters = 3  # TODO make config!
        
        
        pcl_data = self.ros_to_pcl(data)
        fil = pcl_data.make_passthrough_filter()
        fil.set_filter_field_name("z")
        fil.set_filter_limits(0.47, 1.5)
        cloud_filtered = fil.filter()

        # publish filtered cloud 
        self.pub_pcl_filter.publish(self.pcl_to_ros(cloud_filtered))

        #pcl_filter = self.extracting_indices(pcl_data)
        #pcl_msg = self.pcl_to_ros(pcl_filter)
        # self.pub_pcl2.publish(pcl_msg)
        # print("starting segmentation")
        original_size=cloud_filtered.size
        pcl_seg = pcl.PointCloud_PointXYZRGB()

        # remove tabletop plane
        while(cloud_filtered.size>(original_size*40)/100):
            indices, plane_coeff = self.segment_pcl(cloud_filtered,pcl.SACMODEL_PLANE)
            cloud_table = cloud_filtered.extract(indices, negative=False)
            cloud_filtered = cloud_filtered.extract(indices, negative=True) 
        

         
        # get cube planes
        plane_indices=[] 
        cloud_plane = cloud_filtered
        indices, plane_coeff = self.segment_pcl(cloud_plane,pcl.SACMODEL_PLANE)
        cloud_plane = cloud_filtered.extract(indices, negative=False)     

       
        # Do clustering 
        clustered_points=cloud_plane
        # white_cloud = self.XYZRGB_to_XYZ(cloud_filtered) 
        cluster_indices, centroids=k_means_clustering(np.array(cloud_plane),n_clusters,1000)
        # cluster_indices=DBSCAN_clustering(np.array(cloud_plane))
        #check clusetered points
        # print("number of clusters")
        # print(len(cluster_indices)) 


        
        
        # try to make it smart
        if False:
            edge_point_list = []
            for i, cent in enumerate(centroids):
                clustered_points = cloud_plane.extract(cluster_indices[i], negative=False)
                best_point, indices = find_edge(clustered_points, cent)
                edge_point_list.extend(indices)
            print(edge_point_list)
            edge_points = cloud_plane.extract(edge_point_list, negative=False)
            print("edge point")
            print(edge_points)    
            self.pub_pcl_edges.publish(self.pcl_to_ros(edge_points))

        # clustered_points = cloud_plane.extract(cluster_indices[1], negative=False)
        # edge_points,best_point, indices=find_edge(clustered_points,centroids[1])
        
        ####



        # get pose array
        interest_point_list = [] 
        cube_poses=PoseArray()
        for i in range(n_clusters):
             clustered_points = cloud_plane.extract(cluster_indices[i], negative=False)
             edge_points, best_point, indices = find_edge(clustered_points,centroids[i])
             #
             interest_point_list.append(edge_points)
             #
             pose=get_pose(centroids[i],clustered_points[best_point]) 
             cube_poses.poses.append(pose)
        cube_poses.header.frame_id = "base_footprint"
        cube_poses.header.stamp = rospy.Time.now()     
        
        #
        print(interest_point_list)
        #int_oint = interest_point_list[0]
        if False:
            self.pub1_edge.publish(self.pcl_to_ros(interest_point_list[0]))
            self.pub2_edge.publish(self.pcl_to_ros(interest_point_list[1]))
            self.pub3_edge.publish(self.pcl_to_ros(interest_point_list[2]))
            self.pub4_edge.publish(self.pcl_to_ros(interest_point_list[3]))
            self.pub5_edge.publish(self.pcl_to_ros(interest_point_list[4]))
            self.pub6_edge.publish(self.pcl_to_ros(interest_point_list[5]))
            self.pub7_edge.publish(self.pcl_to_ros(interest_point_list[6]))
            self.pub8_edge.publish(self.pcl_to_ros(interest_point_list[7]))
        #

        # publishing
        pcl_seg_msg = self.pcl_to_ros(cloud_plane)
        pcl_clustered=self.pcl_to_ros(edge_points)
       
        # pcl_seg_transf = self.transform_to_base(pcl_seg_msg, stamp)
        self.pub_pcl_planes.publish(pcl_seg_msg)  # cube planes 
        # self.pub2_pcl2.publish(pcl_clustered)  # edge points 
        self.pub3_pose.publish(cube_poses)  # cube poses 
        # self.pub2_pcl2.publish(pcl_seg_transf)
        # self.pub_pcl2.publish(pcl_seg_msg)

    def transform_to_base(self, pc_ros, stamp):

        lookup_time = rospy.get_rostime() 
        # end_time = stamp + rospy.Duration(10)
        target_frame = "base_footprint"  # base_link
        source_frame = "xtion_rgb_optical_frame"
       
        trans = self.tf_buffer.lookup_transform(target_frame, source_frame, lookup_time, rospy.Duration(1))

        cloud_out = do_transform_cloud(pc_ros, trans)
        return cloud_out

    
    def transform_to_cam(self, pc_ros, stamp):

        lookup_time = rospy.get_rostime() 
        # end_time = stamp + rospy.Duration(10)
        target_frame = "xtion_depth_frame"  # base_link
        source_frame = "base_footprint"
       
        trans = self.tf_buffer.lookup_transform(target_frame, source_frame, lookup_time, rospy.Duration(1))

        cloud_out = do_transform_cloud(pc_ros, trans)
        return cloud_out

    def segment_pcl(self, cloud_pcl, model_type):

    
        seg = cloud_pcl.make_segmenter_normals(ksearch=50)
        seg.set_optimize_coefficients(True)
        seg.set_model_type(model_type)
        seg.set_normal_distance_weight(0.1)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_max_iterations(200)
        seg.set_distance_threshold(0.006)
        indices, coefficients = seg.segment()
        
        # https://pointclouds.org/documentation/group__sample__consensus.html
        # describes the coeffici
        if len(indices) == 0:
            print('Could not estimate a planar model for the given dataset.')
            exit(0)

        return indices, coefficients

    def ros_to_pcl(self, ros_cloud):
        # from  https://github.com/jupidity/PCL-ROS-cluster-Segmentation/blob/master/scripts/pcl_helper.py
        """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB
        
            Args:
                ros_cloud (PointCloud2): ROS PointCloud2 message
                
            Returns:
                pcl.PointCloud_PointXYZRGB: PCL XYZRGB point cloud
        """
        points_list = []

        for data in pc2.read_points(ros_cloud, skip_nans=True):
            points_list.append([data[0], data[1], data[2], data[3]])#]

        pcl_data = pcl.PointCloud_PointXYZRGB()
        # pcl_data = pcl.PointCloud()
        pcl_data.from_list(points_list)

        return pcl_data

    def pcl_to_ros(self, pcl_array):
        """ Converts a ROS PointCloud2 message to a pcl PointXYZRGB
        
            Args:
                pcl_array (PointCloud_PointXYZRGB): A PCL XYZRGB point cloud
                
            Returns:
                PointCloud2: A ROS point cloud
        """
        ros_msg = PointCloud2()

        ros_msg.header.stamp = rospy.Time.now()
        ros_msg.header.frame_id = "base_footprint"

        ros_msg.height = 1
        ros_msg.width = pcl_array.size

        ros_msg.fields.append(PointField(
                                name="x",
                                offset=0,
                                datatype=PointField.FLOAT32, count=1))
        ros_msg.fields.append(PointField(
                                name="y",
                                offset=4,
                                datatype=PointField.FLOAT32, count=1))
        ros_msg.fields.append(PointField(
                                name="z",
                                offset=8,
                                datatype=PointField.FLOAT32, count=1))
        ros_msg.fields.append(PointField(
                                name="rgb",
                                offset=16,
                                datatype=PointField.FLOAT32, count=1))

        ros_msg.is_bigendian = False
        ros_msg.point_step = 32
        ros_msg.row_step = ros_msg.point_step * ros_msg.width * ros_msg.height
        ros_msg.is_dense = False
        buffer = []

        for data in pcl_array:
            s = struct.pack('>f', data[3])
            i = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(i).value

            r = (pack & 0x00FF0000) >> 16
            g = (pack & 0x0000FF00) >> 8
            b = (pack & 0x000000FF)

            buffer.append(struct.pack('ffffBBBBIII', data[0], data[1], data[2], 1.0, b, g, r, 0, 0, 0, 0))

        ros_msg.data = "".join(buffer)

        return ros_msg

    def extracting_indices(self, pcl_2):

        print("PointCloud before filtering: " +
            str(pcl_2.width * pcl_2.height) + " data points.")

        # Create the filtering object: downsample the dataset using a leaf size of 1cm
        # pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        # sor.setInputCloud (cloud_blob);
        # sor.setLeafSize (0.01f, 0.01f, 0.01f);
        # sor.filter (*cloud_filtered_blob);
        sor = pcl_2.make_voxel_grid_filter()
        sor.set_leaf_size(0.01, 0.01, 0.01)
        pcl_2_filtered = sor.filter()

        # https://github.com/strawlab/python-pcl/blob/master/pcl/pxi/PointCloud_PointXYZRGB.pxi

        # Convert to the templated PointCloud
        # pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
        # std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
        # cloud_filtered = pcl.PCLPointCloud2(pcl_2_filtered.to_array())
        print('PointCloud after filtering: ' +
            str(pcl_2_filtered.width * pcl_2_filtered.height) + ' data points.')
        return pcl_2_filtered




    