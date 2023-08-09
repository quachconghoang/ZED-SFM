import sys, os
sys.path.append('./Positional_tracking')

import numpy as np
import pyzed.sl as sl
import cv2 as cv

import open3d as o3d
from util import print_camera_information, mat44_to_quaternion, exportTrackingFile, quats_to_matrices

# svo_path = "/home/hoangqc/Datasets/ZED/fence/Fence00_HD.svo"
# area_path = "/home/hoangqc/Datasets/ZED/fence/fence.area"
# save_path = "/home/hoangqc/Datasets/ZED/fence/"

svo_path = "/home/hoangqc/Datasets/ZED/home/Home01_HD.svo"
area_path = "/home/hoangqc/Datasets/ZED/home/home.area"
save_path = "/home/hoangqc/Datasets/ZED/home/"

# svo_path = "/home/hoangqc/Datasets/ZED/building/Building00_HD.svo"
# area_path = "/home/hoangqc/Datasets/ZED/building/building.area"
# save_path = "/home/hoangqc/Datasets/ZED/building/"

# pose_mat44 = exportTrackingFile(svo_path, area_path, save_path, output_file="pose_left_ENU.txt",ros=True)
# pose_mat44 = exportTrackingFile(svo_path, area_path, save_path, output_file="pose_left.txt",ros=False)
# pose_mat44_prev = pose_mat44[::60]

init_params = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720,
                                depth_mode = sl.DEPTH_MODE.ULTRA,
                                coordinate_units=sl.UNIT.METER,
                                coordinate_system=sl.COORDINATE_SYSTEM.IMAGE)
init_params.set_from_svo_file(svo_path)
zed = sl.Camera()
status = zed.open(init_params)
if status != sl.ERROR_CODE.SUCCESS:
    print(repr(status)), exit()

runtime = sl.RuntimeParameters()
camera_pose = sl.Pose()

### set image size
configs = zed.get_camera_information().camera_configuration
image = sl.Mat()
image_raw = sl.Mat()
depth = sl.Mat()
depth_prev = sl.Mat()


image_size = configs.resolution
cam_params = configs.calibration_parameters.left_cam

count = 0
while zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
    if count%10 == 0:
        zed.retrieve_image(image, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
        zed.retrieve_image(image_raw, sl.VIEW.LEFT_UNRECTIFIED, sl.MEM.CPU, image_size)

        # zed.retrieve_image(depth_prev, sl.VIEW.DEPTH, sl.MEM.CPU, image_size)
        # zed.retrieve_measure(depth, sl.MEASURE.DEPTH, sl.MEM.CPU, image_size)

        filename = save_path + "image/" + str(count).zfill(6) + ".jpg"
        cv.imwrite(filename, image.get_data())

        # depth_fname = save_path + "depth_ultra/" + str(count).zfill(6) + ".npy"
        # np.save(depth_fname, depth.get_data())

        # filename = save_path + "image_raw/" + str(count).zfill(6) + ".jpg"
        # cv.imwrite(filename, image_raw.get_data())

        cv.imshow("Image", image_raw.get_data())
        # cv.imshow("Depth", depth_prev.get_data())

        key = cv.waitKey(10)

        if key == ord('q'):
            break

    count += 1

zed.close()
cv.destroyAllWindows()
# np.savetxt(save_path + "pose_left.txt", np.asarray(trajectory))

