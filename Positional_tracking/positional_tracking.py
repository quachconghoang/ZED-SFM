import sys, os
sys.path.append('./Positional_tracking')

import pyzed.sl as sl
import cv2 as cv
import ogl_viewer.tracking_viewer as gl

K_USE_AREA_FILE = True
K_UPDATE_MAP = False

if __name__ == "__main__":

    init_params = sl.InitParameters(camera_resolution=sl.RESOLUTION.HD720,
                                 coordinate_units=sl.UNIT.METER,
                                 coordinate_system=sl.COORDINATE_SYSTEM.IMAGE) # IMAGE
                                 
    # If applicable, use the SVO given as parameter
    # Otherwise use ZED live stream
    if len(sys.argv) == 2:
        filepath = sys.argv[1]
        print("Using SVO file: {0}".format(filepath))
        init_params.set_from_svo_file(filepath)

    zed = sl.Camera()
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()

    tracking_params = sl.PositionalTrackingParameters()
    area_file_path = "./Data/fence.area"
    if ( os.path.isfile(area_file_path) & K_USE_AREA_FILE):
        tracking_params.area_file_path = area_file_path

    zed.enable_positional_tracking(tracking_params)

    runtime = sl.RuntimeParameters()
    camera_pose = sl.Pose()

    camera_info = zed.get_camera_information()
    # Create OpenGL viewer
    viewer = gl.GLViewer()
    viewer.init(camera_info.camera_model)

    py_translation = sl.Translation()
    pose_data = sl.Transform()

    text_translation = ""
    text_rotation = ""

    # Prepare new image size to retrieve half-resolution images
    image = sl.Mat()
    image_size = camera_info.camera_configuration.resolution
    image_size.width = image_size.width /2
    image_size.height = image_size.height /2

    while viewer.is_available():

        if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(image, sl.VIEW.LEFT, sl.MEM.CPU, image_size)
            image_ocv = image.get_data()
            cv.imshow("Image", image_ocv)
            key = cv.waitKey(10)

            tracking_state = zed.get_position(camera_pose)
            if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                rotation = camera_pose.get_rotation_vector()
                translation = camera_pose.get_translation(py_translation)
                text_rotation = str((round(rotation[0], 2), round(rotation[1], 2), round(rotation[2], 2)))
                text_translation = str((round(translation.get()[0], 2), round(translation.get()[1], 2), round(translation.get()[2], 2)))
                pose_data = camera_pose.pose_data(sl.Transform())
            viewer.updateData(pose_data, text_translation, text_rotation, tracking_state)

    viewer.exit()
    cv.destroyAllWindows()

    if(K_USE_AREA_FILE & K_UPDATE_MAP):
        print("Saving area file: {0}".format(area_file_path))
        zed.disable_positional_tracking(area_file_path)
    else:
        zed.disable_positional_tracking()

    zed.close()

