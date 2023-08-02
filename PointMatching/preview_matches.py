import sys, os, glob
import numpy as np
import cv2 as cv

import open3d as o3d
from open3d import camera
from open3d.cuda.pybind.geometry import RGBDImage,PointCloud
from open3d.cuda.pybind.utility import Vector3dVector

import pyzed.sl as sl

from matplotlib import pyplot as plt
from skimage import data, io, filters

from PointMatching.util import quats_to_matrices, getPoint3D


save_path = "/home/hoangqc/Datasets/ZED/fence/"
trajectory = np.loadtxt("/home/hoangqc/Datasets/ZED/fence/pose_left_ENU.txt")
pose_mat = quats_to_matrices(trajectory)
pose_mat = pose_mat[::60]

file_rgb = sorted(glob.glob("/home/hoangqc/Datasets/ZED/fence/image/*.jpg"))
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

def getFrameInfo(id):
    frame = {   'color': io.imread(file_rgb[id]),
                'depth': np.load(file_depth[id]),
                'transform': pose_mat[id],
                'intr': cameraIntrinsic,
                'extr': ros_camExtr     }
    return frame

id = 30
frame_src = getFrameInfo(id)
id = 33
frame_tar = getFrameInfo(id)

src_cloud_rgb = getPoint3D(frame_src)
tar_cloud_rgb = getPoint3D(frame_tar)

src_points = np.asarray(src_cloud_rgb.points)
tar_points = np.asarray(tar_cloud_rgb.points)

o3d.visualization.draw_geometries([axis_pcd,
                                   src_cloud_rgb.remove_non_finite_points(),
                                   tar_cloud_rgb.remove_non_finite_points()],
                                  window_name='pair', width=1024, height=768)

