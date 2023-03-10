import cluster_utils
import pdb
import pcl
# https://github.com/strawlab/python-pcl
from pdb import set_trace as bp
from geometry_msgs.msg import PoseStamped, PoseArray
from agi_vision.msg import PerceptionMSG
import utils
import numpy as np
import ctypes
import struct


class ClusterHandler:
    def __init__(self, n_max, distance, thr_min, thr_max):
        self.n_max = n_max
        self.distance = distance
        self.thr_min = thr_min
        self.thr_max = thr_max

    def __call__(self, pointcloud):
        cube_poses, color_names, confidences, remaining_cloud, extracted_points = self.extract_clusters(pointcloud)
        return cube_poses, color_names, confidences, remaining_cloud, extracted_points
    

    def perception_fixed_clusters(self, pointcloud, n_clusters):
        # for PerceptionMSG
        cube_poses = PoseArray()
        color_names = []
        confidences = []
        model_cloud=utils.generate_source_cloud([0,0,0,100],3500)
        # clustering
        cluster_indices, centroids = utils.k_means_clustering(np.array(pointcloud), n_clusters, 1000)

        for i in range(n_clusters):
            clustered_points = pointcloud.extract(cluster_indices[i],
                                                   negative=False)
            edge_points, best_point, indices = utils.find_edge(
                clustered_points, centroids[i])
            #
            #interest_point_list.append(edge_points)
            #
            cube_poses.poses.append(self._get_pose_icp(model_cloud,clustered_points,centroids[i]))

            # cube_poses.poses.append(self._get_pose(clustered_points, centroids[i]))
            color_names.append(self._get_color(clustered_points))
            confidences.append(self._get_confidence(clustered_points))    
        return cube_poses, color_names, confidences


    def extract_clusters(self, origin_cloud):
        extracted_points = pcl.PointCloud_PointXYZRGB()
        extracted = 0
        print("original size")

        remaining_cloud = origin_cloud
        print(remaining_cloud)

        # for PerceptionMSG
        cube_poses = PoseArray()
        color_names = []
        confidences = []

        for i in range(2):
            current_n = 1
            print("number of clusters")
            print(current_n)
            # old_n = current_n

            # we will overshoot with one cluster so n-1 
            # loop two times !!
            print("adding number of clusters to overshoot")
            one_above_max = True
            all_above_min = True
            while one_above_max and all_above_min:
                current_n += 1
                # clustering
                cluster_indices, centroids = utils.k_means_clustering(np.array(remaining_cloud), current_n, 1000)
                for i in range(current_n):
                   
                    # one cluster is smaller than threshold  
                    if len(cluster_indices[i]) < self.thr_min:
                       all_above_min = False
                       if i == 1:
                           current_n -= 1 
                            
                    
                    # check if all clusters are smaller then thr_max?
                    if len(cluster_indices[i]) > 1300:
                        one_above_max = True
                        break
                    else:
                        one_above_max = False

                if current_n >= self.n_max:
                    do_add = False
                    break

            print("clusters")
            print(current_n)


            print("find and extract good clusters")
            tmp_remaining_cloud = pcl.PointCloud_PointXYZRGB()
            cluster_indices, centroids = utils.k_means_clustering(np.array(remaining_cloud), current_n, 1000)
            print(len(cluster_indices))
            print(len(centroids))
            # whole remaining cloud gets split up
            for i in range(len(cluster_indices)):
                # TODO add color as extra heuristics!
                print(len(cluster_indices[i]))

                if self._no_neighbors(i, centroids) and self._n_points_in_threshold(cluster_indices[i]):
                    # extraction for later vis, not really needed 
                    ext_p = extracted_points.to_array()
                    full_p = np.concatenate((ext_p, remaining_cloud.extract(cluster_indices[i], negative=False).to_array()))
                    extracted_points.from_array(full_p)

                    clustered_points = remaining_cloud.extract(cluster_indices[i], negative=False)
                    # color in remaining cloud i lost!!!!!!!
                    cube_poses.poses.append(self._get_pose(clustered_points, centroids[i]))
                    color_names.append(self._get_color(clustered_points))
                    confidences.append(self._get_confidence(clustered_points))
                    
                    extracted += 1
                    # print("extracted")
                    # print(extracted)
                else:
                    # print("not extracted")
                    remain_p = tmp_remaining_cloud.to_array()
                    remain_full_p = np.concatenate((remain_p, remaining_cloud.extract(cluster_indices[i], negative=False).to_array()))
                    tmp_remaining_cloud.from_array(remain_full_p)

            remaining_cloud = tmp_remaining_cloud
            print(remaining_cloud)
            # reset init_guess
            current_n = 1   
        # print(cluster_indices[i])
        print("number of extracted cubes:")
        print(extracted)
        ######### end of finding iterations ##########

        return cube_poses, color_names, confidences, remaining_cloud, extracted_points


    def _get_pose(self, cluster_pcl, centroid):
        edge_points, best_point, indices = utils.find_edge(
            cluster_pcl, centroid)
        #
        # interest_point_list.append(edge_points)
        #
        pose = utils.get_pose(centroid, cluster_pcl[best_point])
        return pose 

    def _get_pose_icp(self,model_cloud,target_cloud,centroid):
        target_cloud=pcl.PointCloud_PointXYZRGB(np.array(target_cloud)-centroid)
        icp=utils.XYZRGB_to_XYZ(model_cloud).make_IterativeClosestPoint()  
        converged, T, estimate, confidence = icp.icp(
            utils.XYZRGB_to_XYZ(model_cloud),utils.XYZRGB_to_XYZ(target_cloud), max_iter=200) 
        keypoint=np.ones(4)
        keypoint[0:3]= [0,-0.0225,0]
        
        new_centroid=np.ones(4)
        new_centroid[0:3]=np.array([0,0,0])
        keypoint=T.dot(keypoint)+centroid
        new_centroid=T.dot(new_centroid)+centroid  
        pose=utils.get_pose_icp(new_centroid,keypoint)
        return pose

    def _get_color(self, cluster_pcl):
        color_scores, color_names = self._unpack_color(cluster_pcl)

        max_value = max(color_scores)
        index = color_scores.index(max_value)
        res = color_names[index]
        numbers = np.array(color_scores)
        # Check if all values in array are zeros
        if np.all(numbers == 0) and len(color_scores) > 0:
            res = "no_color_found"
        return res 

    def _unpack_color(self, cluster_pcl):
        clustered_array = cluster_pcl.to_array()
        names = ["red", "blue", "green", "yellow"]
        r_all = g_all = b_all = y_all = 0

        for data in clustered_array:
            s = struct.pack('>f', data[3])
            k = struct.unpack('>l', s)[0]
            pack = ctypes.c_uint32(k).value
            r = (pack & 0x00FF0000) >> 16
            g = (pack & 0x0000FF00) >> 8
            b = (pack & 0x000000FF)

            # heuristics
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

        return [r_all, b_all, g_all, y_all], names


    def _get_confidence(self, cluster_pcl):
        # TODO add ICP or some heuristic for confidence
        return 1.0


    # conditions for a "good" cluster 
    def _only_one_color(self, cluster_pcl):
        color_scores, color_names = self._unpack_color(cluster_pcl)
        if True:
            return True
        else:
            return False

    def _no_neighbors(self, candidate, centroids): 
        for i in range(len(centroids)):
            # do nor compare to condidate cluster itself 
            if i != candidate:
                distance = np.linalg.norm(centroids[candidate][:3] -
                                            centroids[i][:3])
                if distance < self.distance:
                    print("no neighbors False")
                    return False
        return True

    def _n_points_in_threshold(self, clust_indi):
        if len(clust_indi) < self.thr_max and len(clust_indi) > self.thr_min:
            return True
        else:
            print("n_points_in_threshold False")
            return False 