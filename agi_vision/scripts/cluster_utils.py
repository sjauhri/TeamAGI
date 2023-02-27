import numpy as np
from sklearn.cluster import KMeans, DBSCAN
import pcl
from geometry_msgs.msg import Pose
import rospy
from tf.transformations import quaternion_from_euler

import pdb
from pdb import set_trace as bp

# kmeans clustering    
def k_means_clustering(data,n_clusters,max_iterations):
    kmeans = KMeans(n_clusters=n_clusters, random_state=0, max_iter=max_iterations).fit(data)
    clustered_indices=[]
    for i in range(n_clusters):
            clustered_indices.append(np.where(kmeans.labels_==i)[0])
    return clustered_indices, kmeans.cluster_centers_


def find_n_clusters_2(origin_cloud, n_init_guess, n_max, thr_min, thr_max, n_iterate):

    # conditions for a "good" cluster 
    # TODO add color condition 
    def only_one_color(points):
        if True:
            return True
        else:
            return False

    def no_neighbors(candidate, centroids): 
        for i in range(len(centroids)):
            # do nor compare to condidate cluster itself 
            if i != candidate:
                distance = np.linalg.norm(centroids[candidate][:3] -
                                              centroids[i][:3])
                if distance < 0.05:
                    return False
        return True

    def n_points_in_threshold(clust_indi, thr_min, thr_max):
        if len(clust_indi) < thr_max and len(clust_indi) > thr_min:
            return True
        else:
            return False 

    extracted_points = pcl.PointCloud_PointXYZRGB()
    extracted = 0
    print("original size")
    print(origin_cloud)
    remaining_cloud = origin_cloud

    # for PerceptionMSG


    for i in range(n_iterate):
        current_n = n_init_guess - 1
        print("number of clusters")
        print(current_n)
        # old_n = current_n

        print("adding number of clusters to overshoot")
        do_add = True
        while do_add:
            current_n += 1
            # clustering
            cluster_indices, centroids = k_means_clustering(np.array(remaining_cloud), current_n, 1000)
            for i in range(current_n):
                
                # cluster is small 
                if len(cluster_indices[i]) < thr_min:
                    do_add = False

                if current_n >= n_max:
                    do_add = False
                    break

        print("clusters")
        print(current_n)

        print("find and extract good clusters")
        tmp_remaining_cloud = pcl.PointCloud_PointXYZRGB()
        # whole remaining cloud gets split up
        for i in range(len(cluster_indices)):

            # TODO add color as extra heuristics!
            if no_neighbors(i, centroids) and n_points_in_threshold(cluster_indices[i], thr_min, thr_max):

                ext_p = extracted_points.to_array()
                full_p = np.concatenate((ext_p, remaining_cloud.extract(cluster_indices[i], negative=False).to_array()))
                extracted_points.from_array(full_p)

           
                # cloud_plane = cloud_plane.extract(cluster_indices[i], negative=True)
                
                extracted += 1
                print("extracted")
                print(extracted)
            else:
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


    # if finding double cluster on cube:
    #    only look at those poiints!


    # remaining_points = cloud_plane

    # TODO publish on pcl handler for debug and vis 
    return extracted, remaining_cloud, extracted_points


def find_n_clusters_1(cloud_plane):
    current_n = 5
    thr_max = 1500
    thr_min = 500
    running = True
    while running:
        print("number of clusters")
        print(current_n)
        old_n = current_n
        # clustering
        cluster_indices, centroids = k_means_clustering(np.array(cloud_plane), current_n, 1000)

        for i in range(current_n):
            # clustered_points = cloud_plane.extract(cluster_indices[i], negative=False)

            # cluster it so small 
            if len(cluster_indices[i]) < thr_min:
                print("Cluster is very small!")

            # cluster is to big, split it up 
            elif len(cluster_indices[i]) > thr_max:
                print("Cluster is too big!")
                too_much = 0 
                current_n +=1
                break

        if old_n == current_n:
            # distance between centroids should be more than 5 cm
            # print(centroids)
            for j in range(i, len(centroids), 1):
                too_much = 0
                # print(j)
                # dont compare same cluster 
                if i != j:
                    distance = np.linalg.norm(centroids[i][:3]-centroids[j][:3])

                    if distance < 0.03:
                        too_much += 1
                        print("Clusters are at the same cube!")
                        print(distance)
                        print(i)
                        print(j)

            n_clusters = current_n - too_much
            running = False

    print("final number of clusters from find clusters 1")
    print(n_clusters)
    return n_clusters