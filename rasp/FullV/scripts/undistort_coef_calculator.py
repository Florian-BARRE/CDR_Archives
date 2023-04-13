# Calculate distort coef from chess pictures and save the coef
from classes.calibration_camera.chess_calibration_camera import Camera_Calibrator
import os

path = os.path.join("classes", "calibration_camera", "raspicam_calib_pictures")
cam_calibrator = Camera_Calibrator(path, (4, 6))

if cam_calibrator.init_calibration_cam(save_recognize_picture_copy=True):
    cam_calibrator.save_undistort_coef(
        os.path.join("classes", "calibration_camera", "distortion_coef", "raspicam_undistort_coef")
    )
