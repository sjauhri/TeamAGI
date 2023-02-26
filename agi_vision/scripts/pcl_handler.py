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

from agi_vision.msg import PerceptionMSG


class PCLHandler:

    def __init__(self):
        self.node = rospy.init_node('pcl_handling', anonymous=True)

        self.sub = rospy.Subscriber('/xtion/depth_registered/points',
                                    PointCloud2, self.callback)
        self.pub_pcl_filter = rospy.Publisher('Pointcloud2_filtered',
                                              PointCloud2,
                                              queue_size=10)
        self.pub_pcl_planes = rospy.Publisher('Pointcloud2_cube_planes',
                                              PointCloud2,
                                              queue_size=10)
        self.pub_pcl_edges = rospy.Publisher('Edge_points',
                                             PointCloud2,
                                             queue_size=10)

        self.pub_perception = rospy.Publisher('PerceptionMSG',
                                              PerceptionMSG,
                                              queue_size=10)

        self.pub1_edge = rospy.Publisher('Edge_point1',
                                         PointCloud2,
                                         queue_size=10)
        self.pub2_edge = rospy.Publisher('Edge_point2',
                                         PointCloud2,
                                         queue_size=10)
        self.pub3_edge = rospy.Publisher('Edge_point3',
                                         PointCloud2,
                                         queue_size=10)
        self.pub4_edge = rospy.Publisher('Edge_point4',
                                         PointCloud2,
                                         queue_size=10)
        self.pub5_edge = rospy.Publisher('Edge_point5',
                                         PointCloud2,
                                         queue_size=10)
        self.pub6_edge = rospy.Publisher('Edge_point6',
                                         PointCloud2,
                                         queue_size=10)
        self.pub7_edge = rospy.Publisher('Edge_point7',
                                         PointCloud2,
                                         queue_size=10)
        self.pub8_edge = rospy.Publisher('Edge_point8',
                                         PointCloud2,
                                         queue_size=10)

        self.pub3_pose = rospy.Publisher('cube_poses',
                                         PoseArray,
                                         queue_size=10)
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
        data = self.transform_to_base(data, stamp)
        # for p in pc2.read_points(data, field_names = ("x", "y", "z"), skip_nans=True):
        #     print " x : %f  y: %f  z: %f" %(p[0],p[1],p[2])

        n_clusters_fixed = 3  # TODO make config!

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
        original_size = cloud_filtered.size
        pcl_seg = pcl.PointCloud_PointXYZRGB()

        # remove tabletop plane
        while (cloud_filtered.size > (original_size * 40) / 100):
            indices, plane_coeff = self.segment_pcl(cloud_filtered,
                                                    pcl.SACMODEL_PLANE)
            cloud_table = cloud_filtered.extract(indices, negative=False)
            cloud_filtered = cloud_filtered.extract(indices, negative=True)

        # get cube planes
        plane_indices = []
        cloud_plane = cloud_filtered
        indices, plane_coeff = self.segment_pcl(cloud_plane,
                                                pcl.SACMODEL_PLANE)
        cloud_plane = cloud_filtered.extract(indices, negative=False)

        # find n_cluster!
        find_n_cluster = True
        n_clusters = n_clusters_fixed

        if find_n_cluster:
            current_n = 1
            thr_max = 1500
            thr_min = 500
            running = True
            while running:
                print("number of clusters")
                print(current_n)
                old_n = current_n

                # clustering
                cluster_indices, centroids = k_means_clustering(
                    np.array(cloud_plane), current_n, 1000)

                for i in range(current_n):
                    # clustered_points = cloud_plane.extract(cluster_indices[i], negative=False)

                    # cluster it so small
                    if len(cluster_indices[i]) < thr_min:
                        print("Cluster is very small!")

                    # cluster is to big, split it up
                    elif len(cluster_indices[i]) > thr_max:
                        print("Cluster is too big!")
                        too_much = 0
                        current_n += 1
                        break

                    # cluster between min and max
                    else:  # len(cluster_indices[i]) < thr_max:

                        # point distance stuff
                        # clustered_points = cloud_plane.extract(cluster_indices[i], negative=False)
                        # best_point, indices = find_edge(clustered_points, centroids[i])
                        # array_cloud=np.array(XYZRGB_to_XYZ(points))
                        # center=centroids[i][0:3]
                        # distances=np.linalg.norm(array_cloud-center,axis=1)

                        # distcnace between centroids should be more than 5 cm
                        # print(centroids)
                        for j in range(i, len(centroids), 1):
                            too_much = 0
                            # print(j)
                            # dont compare same cluster
                            if i != j:
                                distance = np.linalg.norm(centroids[i][:3] -
                                                          centroids[j][:3])

                                if distance < 0.03:
                                    too_much += 1
                                    print("Clusters are at the same cube!")
                                    print(distance)
                                    print(i)
                                    print(j)

                if old_n == current_n:
                    n_clusters = current_n - too_much
                    running = False

            print("final number of clusters")
            print(n_clusters)

        # Do clustering
        clustered_points = cloud_plane

        if n_clusters == n_clusters_fixed:
            print("found exact number of clusters!")
        else:
            print("find cluster amount failed!")
            print("number of clusters found is")
            print(n_clusters)
            print("number of real clusters is")
            print(n_clusters_fixed)
            print("using n_clusters:")
            # TODO: turn off to use found number of clusters
            n_clusters = n_clusters_fixed
            print(n_clusters)

        # white_cloud = self.XYZRGB_to_XYZ(cloud_filtered)
        cluster_indices, centroids = k_means_clustering(
            np.array(cloud_plane), n_clusters, 1000)
        # cluster_indices=DBSCAN_clustering(np.array(cloud_plane))
        #check clusetered points
        # print("number of clusters")
        # print(len(cluster_indices))

        # try to make it smart
        if False:
            edge_point_list = []
            for i, cent in enumerate(centroids):
                clustered_points = cloud_plane.extract(cluster_indices[i],
                                                       negative=False)
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
        cube_poses = PoseArray()
        color_list = []

        for i in range(n_clusters):
            clustered_points = cloud_plane.extract(cluster_indices[i],
                                                   negative=False)
            print(clustered_points)
            edge_points, best_point, indices = find_edge(
                clustered_points, centroids[i])
            #
            interest_point_list.append(edge_points)
            #
            pose = get_pose(centroids[i], clustered_points[best_point])
            cube_poses.poses.append(pose)

            #########################################
            # color detection for pointcloud-cluster
            clustered_array = clustered_points.to_array()
            r_all = 0
            g_all = 0
            b_all = 0
            y_all = 0

            for data in clustered_array:
                res = "not found"
                s = struct.pack('>f', data[3])
                k = struct.unpack('>l', s)[0]
                pack = ctypes.c_uint32(k).value

                r = (pack & 0x00FF0000) >> 16
                g = (pack & 0x0000FF00) >> 8
                b = (pack & 0x000000FF)

                if r > (b + 50) and g > (b + 50):
                    y_all += 1
                else:
                    if r > (g + 50) and r > (b + 50):
                        r_all += 1
                    if g > (r + 30) and g > (b + 10):
                        g_all += 1
                    if b > (r + 25):
                        b_all += 1

            # print(r_all)
            # print(g_all)
            # print(b_all)
            # print(y_all)

            input_list = [r_all, b_all, g_all, y_all]
            max_value = max(input_list)
            index = input_list.index(max_value)
            names = ["red", "blue", "green", "yellow"]
            res = names[index]
            numbers = np.array(input_list)
            # Check if all values in array are zeros
            if np.all(numbers == 0) and len(input_list) > 0:
                res = "no_color_found"
            color_list.append(res)

            ########### end of loop

        print(color_list)

        cube_poses.header.frame_id = "base_footprint"
        cube_poses.header.stamp = rospy.Time.now()

        #
        # print(interest_point_list)
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
        pcl_clustered = self.pcl_to_ros(edge_points)

        # pcl_seg_transf = self.transform_to_base(pcl_seg_msg, stamp)
        self.pub_pcl_planes.publish(pcl_seg_msg)  # cube planes
        # self.pub2_pcl2.publish(pcl_clustered)  # edge points
        self.pub3_pose.publish(cube_poses)  # cube poses

        perc_msg = PerceptionMSG()
        perc_msg.size = len(color_list)
        perc_msg.pose_array = cube_poses
        perc_msg.color_names = color_list
        perc_msg.confidence = [1.0 for i in range(len(color_list))]
        self.pub_perception.publish(perc_msg)

        # self.pub2_pcl2.publish(pcl_seg_transf)
        # self.pub_pcl2.publish(pcl_seg_msg)

    def transform_to_base(self, pc_ros, stamp):

        lookup_time = rospy.get_rostime()
        # end_time = stamp + rospy.Duration(10)
        target_frame = "base_footprint"  # base_link
        source_frame = "xtion_rgb_optical_frame"

        trans = self.tf_buffer.lookup_transform(target_frame, source_frame,
                                                lookup_time, rospy.Duration(1))

        cloud_out = do_transform_cloud(pc_ros, trans)
        return cloud_out

    def transform_to_cam(self, pc_ros, stamp):

        lookup_time = rospy.get_rostime()
        # end_time = stamp + rospy.Duration(10)
        target_frame = "xtion_depth_frame"  # base_link
        source_frame = "base_footprint"

        trans = self.tf_buffer.lookup_transform(target_frame, source_frame,
                                                lookup_time, rospy.Duration(1))

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
            points_list.append([data[0], data[1], data[2], data[3]])  #]

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

        ros_msg.fields.append(
            PointField(name="x",
                       offset=0,
                       datatype=PointField.FLOAT32,
                       count=1))
        ros_msg.fields.append(
            PointField(name="y",
                       offset=4,
                       datatype=PointField.FLOAT32,
                       count=1))
        ros_msg.fields.append(
            PointField(name="z",
                       offset=8,
                       datatype=PointField.FLOAT32,
                       count=1))
        ros_msg.fields.append(
            PointField(name="rgb",
                       offset=16,
                       datatype=PointField.FLOAT32,
                       count=1))

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

            buffer.append(
                struct.pack('ffffBBBBIII', data[0], data[1], data[2], 1.0, b,
                            g, r, 0, 0, 0, 0))

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
              str(pcl_2_filtered.width * pcl_2_filtered.height) +
              ' data points.')
        return pcl_2_filtered
