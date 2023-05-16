# Load undistort coef
from classes.calibration_camera.chess_calibration_camera import Camera_Calibrator
import os

path = os.path.join("classes", "calibration_camera", "distortion_coef", "raspicam_undistort_coef.json")
cam_calibrator = Camera_Calibrator("", (0, 0))
cam_calibrator.load_undistort_coef(path)
print()