import sys, os, glob
import numpy as np
import cv2 as cv

import open3d as o3d
from open3d import camera
from open3d.cuda.pybind.geometry import RGBDImage,PointCloud
from open3d.cuda.pybind.utility import Vector3dVector

import pyzed.sl as sl

from PointMatching.util import quats_to_matrices


svo_path = "/home/hoangqc/Datasets/ZED/fence/Fence00_HD.svo"
area_path = "/home/hoangqc/Datasets/ZED/fence/fence.area"
save_path = "/home/hoangqc/Datasets/ZED/fence/"
# numpy load from txt file
trajectory = np.loadtxt("/home/hoangqc/Datasets/ZED/fence/pose_left_ENU.txt")
pose_mat = quats_to_matrices(trajectory)
pose_mat = pose_mat[::60]

file_rgb = sorted(glob.glob("/home/hoangqc/Datasets/ZED/fence/image/*.png"))
file_depth = sorted(glob.glob("/home/hoangqc/Datasets/ZED/fence/depth_ultra/*.npy"))

config_cam = {'width':1280, 'height':720,
              'fx':522.5470581054688, 'fy':522.5470581054688,
              'cx':645.46435546875, 'cy':361.37939453125}
cameraIntrinsic = camera.PinholeCameraIntrinsic(**config_cam)

ros_camExtr = np.array([[0, -1, 0, 0],
                    [0, 0, -1, 0],
                    [1, 0, 0, 0],
                    [0, 0, 0, 1]], dtype=np.float64)

axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
img_cloud = o3d.geometry.PointCloud()

vis = o3d.visualization.VisualizerWithKeyCallback()
vis.create_window(window_name='Trajectory', width=1024, height=768)
vis.add_geometry(axis_pcd)
vis.add_geometry(img_cloud)
vis.poll_events();vis.update_renderer()
id = 0
for pose in pose_mat:
    cam_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2, origin=[0, 0, 0])
    cam_pcd.transform(pose)
    vis.add_geometry(cam_pcd)

    img_color = cv.imread(file_rgb[id])
    img_depth = np.load(file_depth[id])
    img_trans = pose_mat[id]
    img_rgbd = RGBDImage.create_from_color_and_depth(
            color=o3d.geometry.Image(img_color),
            depth=o3d.geometry.Image(img_depth),
            depth_scale=1.0, depth_trunc=np.inf,
            convert_rgb_to_intensity=True)
    cloud = PointCloud.create_from_rgbd_image(image=img_rgbd,
                                            intrinsic=cameraIntrinsic,
                                            extrinsic=ros_camExtr)
    cloud.transform(pose)
    img_cloud.points = cloud.points
    img_cloud.colors = cloud.colors

    vis.update_geometry(img_cloud)
    vis.poll_events();
    vis.update_renderer()
    id +=1
    ...

vis.poll_events();vis.update_renderer();vis.run()
vis.destroy_window()
