import numpy as np
from sklearn.cluster import KMeans, DBSCAN


# kmeans clustering    
def k_means_clustering(data,n_clusters,max_iterations):
    kmeans = KMeans(n_clusters=n_clusters, random_state=0, max_iter=max_iterations).fit(data)
    clustered_indices=[]
    for i in range(n_clusters):
            clustered_indices.append(np.where(kmeans.labels_==i)[0])
    return clustered_indices



# XYZRGB PointCloud to XYZ PointCloud
def XYZRGB_to_XYZ(pcl_data):
    pcl_array=np.array(pcl_data)
    pcl_XYZ_list=(pcl_array[:,0:3]).tolist()  
    pcl_XYZ = pcl.PointCloud()
    pcl_XYZ.from_list(pcl_XYZ_list)
    return pcl_XYZ

# Euclidean Clustering 
def cluster(LEAF_SIZE, cloud_points, Max_Cluster):
    # # Do segmentation
    white_cloud = XYZRGB_to_XYZ(cloud_points)
    tree = white_cloud.make_kdtree()
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold
    # as well as minimum and maximum cluster size (in points)
    ec.set_ClusterTolerance(LEAF_SIZE*2)
    ec.set_MinClusterSize(20)
    ec.set_MaxClusterSize(Max_Cluster)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()
    clustered_points=pcl.PointCloud_PointXYZRGB()
       
        
    return cluster_indices




# DBSCAN
def DBSCAN_clustering(data):
    dbs = DBSCAN(min_samples=3).fit(data)
    clustered_indices=[]
    n_clusters=np.max(optics.labels_)
    for i in range(n_clusters):
            clustered_indices.append(np.where(optics.labels_==i)[0])
    return clustered_indices        