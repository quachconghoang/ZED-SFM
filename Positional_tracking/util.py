import numpy as np
import pyzed.sl as sl
from scipy.spatial.transform import Rotation as R

def print_camera_information(cam):
    configs = cam.get_camera_information().camera_configuration
    print("Distorsion factor of the right cam before calibration: {0}.".format(configs.calibration_parameters_raw.right_cam.disto))
    print("Distorsion factor of the right cam after calibration: {0}.\n".format(configs.calibration_parameters.right_cam.disto))

    print("Confidence threshold: {0}".format(cam.get_runtime_parameters().confidence_threshold))
    print("Depth min and max range values: {0}, {1}".format(cam.get_init_parameters().depth_minimum_distance,
                                                            cam.get_init_parameters().depth_maximum_distance)
          )
    print("Resolution: {0}, {1}.".format(round(configs.resolution.width, 2),
                                         configs.resolution.height))
    print("Camera FPS: {0}".format(configs.fps))
    print("Frame count: {0}.\n".format(cam.get_svo_number_of_frames()))


def exportTrackingFile(svo_path, area_path, save_path, output_file="pose_left.txt", ros=False):
    coord=sl.COORDINATE_SYSTEM.IMAGE
    if ros:
        coord=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD

    init_params = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720,
                                    coordinate_units=sl.UNIT.METER,
                                    coordinate_system=coord)

    init_params.set_from_svo_file(svo_path)
    zed = sl.Camera()
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status)), exit()

    runtime = sl.RuntimeParameters()
    camera_pose = sl.Pose()

    ### Init tracking
    tracking_params = sl.PositionalTrackingParameters()
    tracking_params.area_file_path = area_path
    zed.enable_positional_tracking(tracking_params)

    trajectory = []
    while zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
        quad = mat44_to_quaternion(np.eye(4))
        tracking_state = zed.get_position(camera_pose)
        if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
            quad = mat44_to_quaternion(camera_pose.pose_data().m)
        trajectory.append(quad)

    zed.disable_positional_tracking()
    zed.close()
    np.savetxt(save_path + output_file, np.asarray(trajectory))


# transform matrix 44 to quaternion
def mat44_to_quaternion(mat44):
    rot = mat44[:3, :3]
    quat = R.from_matrix(rot).as_quat()
    trans = mat44[:3, 3]

    return np.concatenate((trans, quat))
    # r = R.from_matrix(transform)
    # return r.as_quat()

def quats_to_matrices(quat_datas):
    data_len = quat_datas.shape[0]
    SEs = []
    for quat in quat_datas:
        SO = R.from_quat(quat[3:7]).as_matrix()
        SE = np.eye(4)
        SE[0:3, 0:3] = SO
        SE[0:3, 3] = quat[0:3]
        SEs.append(SE)
    return SEs