# Load undistort coef and try to undistort image
from classes.camera import Camera
from classes.calibration_camera.chess_calibration_camera import Camera_Calibrator
import os
import cv2

webcam = Camera()
path = os.path.join("classes", "calibration_camera", "distortion_coef", "raspicam_undistort_coef.json")
cam_calibrator = Camera_Calibrator("", (0, 0))
cam_calibrator.load_undistort_coef(path)

while True:
    before_frame = webcam.Read()
    after_frame = cam_calibrator.undistort_image(before_frame)
    cv2.imwrite(os.path.join("captures", "BEFORE.jpg"), before_frame)
    cv2.imwrite(os.path.join("captures", "AFTER.jpg"), after_frame)


