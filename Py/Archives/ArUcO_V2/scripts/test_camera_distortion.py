from classes.camera import Camera
from classes.calibration_cameraV2.chess_calibration_camera import Camera_Calibrator
from os import path
import cv2

webcam = Camera(res_w=1280)
path = path.join("classes", "calibration_cameraV2", "test_simple")
cam_calibrator = Camera_Calibrator(path, (5, 7))

if cam_calibrator.init_calibration_cam(save_recognize_picture_copy=True):
    while True:
        before_frame = webcam.Read()
        after_frame = cam_calibrator.undistort_image(before_frame)
        webcam.Update_Monitor(after_frame)
        cv2.imshow("BEFORE", before_frame)
        cv2.imshow("AFTER", after_frame)




