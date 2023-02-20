import pickle
import numpy as np
'''
6D maps: "https://archimedes.ias.informatik.tu-darmstadt.de/s/5nnZRXSjAzGG8Hn"
1) pkl_p3TOp2(): Modify the pkl file generated by python3 env to make it available for python2
2) clip_maps(height, boundary): clip the map with boundary along z axis
'''


def pkl_p3TOp2():
    # Modify the pkl file generated by python3 to make it available for python2
    path_maps_6D_p3_l = "/home/qiao/Proj_Robman/maps/maps_6D_p3/filt_reach_map_gripper_left_grasping_frame_torso_False.pkl"
    path_maps_6D_p3_r = "/home/qiao/Proj_Robman/maps/maps_6D_p3/filt_reach_map_gripper_right_grasping_frame_torso_False.pkl"
    map_l = pickle.load(open(path_maps_6D_p3_l, 'rb'))
    map_r = pickle.load(open(path_maps_6D_p3_r, 'rb'))
    path_maps_6D_p2_l = "/home/qiao/Proj_Robman/maps/maps_6D_p2/filt_reach_map_gripper_left_grasping_frame_torso_False.pkl"
    path_maps_6D_p2_r = "/home/qiao/Proj_Robman/maps/maps_6D_p2/filt_reach_map_gripper_right_grasping_frame_torso_False.pkl"
    pickle.dump(map_l, open(path_maps_6D_p2_l, "wb"), protocol=2)
    pickle.dump(map_r, open(path_maps_6D_p2_r, "wb"), protocol=2)


def clip_map_axes(map, xyz, boundary):

    # clipped map along x axis
    x = map[:, 0]
    inds_x1 = np.argwhere(x < (xyz[0] + boundary[0])).reshape(-1)
    map_cl_x1 = map[inds_x1, :]
    inds_x2 = np.argwhere(map_cl_x1[:, 0] > (xyz[0] - boundary[0])).reshape(-1)
    map_cl_x2 = map[inds_x1[inds_x2], :]

    # clipped map along y axis
    y = map_cl_x2[:, 1]
    inds_y1 = np.argwhere(y < (xyz[1] + boundary[1])).reshape(-1)
    map_cl_y1 = map_cl_x2[inds_y1, :]
    inds_y2 = np.argwhere(map_cl_y1[:, 1] > (xyz[1] - boundary[1])).reshape(-1)
    map_cl_y2 = map_cl_x2[inds_y1[inds_y2], :]

    # clipped map along z axis
    z = map_cl_y2[:, 2]
    inds_z1 = np.argwhere(z < (xyz[2] + boundary[2])).reshape(-1)
    map_cl_z1 = map_cl_y2[inds_z1, :]
    inds_z2 = np.argwhere(map_cl_z1[:, 2] > (xyz[2] - boundary[2])).reshape(-1)
    map_cl_z2 = map_cl_y2[inds_z1[inds_z2], :]

    map_cl = map_cl_z2
    # map_cl[:, :-2] = np.ceil(map_cl[:, :-2] * 100) / 100

    print("map shape:", map.shape)

    # print("x min:", np.min(map[:,0]), "->", np.min(map_cl_x2[:, 0]))
    # print("x max:", np.max(map[:,0]), "->", np.max(map_cl_x2[:, 0]))
    # print(map_cl_x2.shape)

    # print("y min:", np.min(map[:, 1]), "->", np.min(map_cl_y2[:, 1]))
    # print("y max:", np.max(map[:, 1]), "->", np.max(map_cl_y2[:, 1]))
    # print(map_cl_y2.shape)

    # print("z min:", np.min(map[:,2]), "->", np.min(map_cl_z2[:, 2]))
    # print("z max:", np.max(map[:,2]), "->", np.max(map_cl_z2[:, 2]))
    print("clipped map shape", map_cl.shape)
    print("---------------------")

    return map_cl


def clip_maps(xyz, boundary):

    path_maps_6D_p2_l = "/home/qiao/Proj_Robman/maps/maps_6D_p2/filt_reach_map_gripper_left_grasping_frame_torso_False.pkl"
    path_maps_6D_p2_r = "/home/qiao/Proj_Robman/maps/maps_6D_p2/filt_reach_map_gripper_right_grasping_frame_torso_False.pkl"
    map_l = pickle.load(open(path_maps_6D_p2_l, 'rb'))
    map_r = pickle.load(open(path_maps_6D_p2_r, 'rb'))

    map_clip_l = clip_map_axes(map_l, xyz, boundary)
    map_clip_r = clip_map_axes(map_r, xyz, boundary)

    map_l_cliped = map_clip_l
    map_r_cliped = map_clip_r
    path_reach_map_6D_left_clipped = "/home/qiao/Proj_Robman/maps/maps_6D_p2_clipped/reach_map_6D_left_clipped.pkl"
    path_reach_map_6D_right_clipped = "/home/qiao/Proj_Robman/maps/maps_6D_p2_clipped/reach_map_6D_right_clipped.pkl"
    pickle.dump(map_l_cliped,
                open(path_reach_map_6D_left_clipped, "wb"),
                protocol=2)
    pickle.dump(map_r_cliped,
                open(path_reach_map_6D_right_clipped, "wb"),
                protocol=2)


if __name__ == '__main__':
    # pkl_p3TOp2()S
    cube = 0.045
    xyz = [0, 0, 0.45 + 0.03]
    boundary = [(0.75 + 0.03) / 2 + 0.1, (0.75 + 0.03) / 2 + 0.1,
                0.2]  # redundant boundary for axes
    # left: x:-

    clip_maps(xyz, boundary)
