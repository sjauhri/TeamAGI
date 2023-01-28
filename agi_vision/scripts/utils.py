import numpy as np
from sklearn.cluster import KMeans, DBSCAN
import pcl
from geometry_msgs.msg import PoseStamped
import rospy
# kmeans clustering    
def k_means_clustering(data,n_clusters,max_iterations):
    kmeans = KMeans(n_clusters=n_clusters, random_state=0, max_iter=max_iterations).fit(data)
    clustered_indices=[]
    for i in range(n_clusters):
            clustered_indices.append(np.where(kmeans.labels_==i)[0])
    return clustered_indices, kmeans.cluster_centers_

# hierarchy clustering



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



# XYZRGB PointCloud to XYZ PointCloud
def XYZRGB_to_XYZ(pcl_data):
    pcl_array=np.array(pcl_data)
    pcl_XYZ_list=(pcl_array[:,0:3]).tolist()  
    pcl_XYZ = pcl.PointCloud()
    pcl_XYZ.from_list(pcl_XYZ_list)
    return pcl_XYZ

def find_edge(points,centroid):
    white_cloud=np.array(XYZRGB_to_XYZ(points))
    center=centroid[0:3]
    distances=np.linalg.norm(white_cloud-center,axis=1)
    centroid_idx=np.argmin(distances)
    distances[np.where(distances>0.0321)]=0
    indices=np.where(distances>0.031) 
    best_point=np.argmax(distances)
    # indices=indices[0].tolist()
    # indices.append(centroid_idx)
    indices=[]
    indices.append(best_point)
    indices.append(centroid_idx)
    edge_points=points.extract(indices, negative=False)
    return edge_points,best_point


def get_pose(A, B):
    A=np.array(A)[0:3]
    B=np.array(B)[0:3]
    a = np.cross(A, B)
    x = a[0]
    y = a[1]
    z = a[2]
    A_length = np.linalg.norm(A)
    B_length = np.linalg.norm(B)
    w = np.sqrt((A_length ** 2) * (B_length ** 2)) + np.dot(A, B)

    norm = np.sqrt(x ** 2 + y ** 2 + z ** 2 + w ** 2)
    if norm == 0:
        norm = 1

    x /= norm
    y /= norm
    z /= norm
    w /= norm


    pose = PoseStamped()
    pose.header.frame_id = "base_footprint"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = A[0]
    pose.pose.position.y = A[1]
    pose.pose.position.z = A[2]
    pose.pose.orientation.x = x
    pose.pose.orientation.y = y
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w

    return pose