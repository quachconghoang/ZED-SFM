import sys, os
sys.path.append('./Positional_tracking')

import numpy as np
import pyzed.sl as sl
import cv2 as cv
import open3d as o3d
from Positional_tracking.util import print_camera_information, mat44_to_quaternion, exportTrackingFile, quats_to_matrices

svo_path = "/home/hoangqc/Datasets/ZED/fence/Fence00_HD.svo"
area_path = "/home/hoangqc/Datasets/ZED/fence/fence.area"
save_path = "/home/hoangqc/Datasets/ZED/fence/"
# numpy load from txt file
trajectory = np.loadtxt("../Data/pose_left_ENU.txt")
pose_mat = quats_to_matrices(trajectory)

fx = 522.5470581054688
fy = 522.5470581054688
cx = 645.46435546875
cy = 361.37939453125

axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])

vis = o3d.visualization.VisualizerWithKeyCallback()
vis.create_window(window_name='Trajectory', width=1024, height=768)
vis.add_geometry(axis_pcd)
vis.poll_events();vis.update_renderer()

new_pose_mat = pose_mat[::60]
for pose in new_pose_mat:
    cam_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
    cam_pcd.transform(pose)
    vis.add_geometry(cam_pcd)
    vis.poll_events();
    vis.update_renderer()
    ...

vis.poll_events();vis.update_renderer();vis.run()
vis.destroy_window()
