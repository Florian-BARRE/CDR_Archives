import math
import cv2

from classes.camera import Camera
from classes.arucoRecognizer import ArucoRecognizer, draw_markers, draw_corners, draw_barycenter
from calibration_camera.chess_calibration_camera import Camera_Calibrator

webcam = Camera()
aruco_ia = ArucoRecognizer("DICT_4X4_100")

cam_calibrator = Camera_Calibrator("./calibration_camera/calibration_pictures/cal2_sol.jpg", (12, 18))
cam_calibrator.init_calibration_cam()

while True:
    frame = webcam.Read()
    frame = cam_calibrator.undistort_image(frame)

    founds = aruco_ia.detect(frame)
    #draw_barycenter(frame, founds)
    marked_frame = draw_markers(frame, founds)

    for index in range(founds.len):
        # Calcul Xcenter
        x = (int(founds.corners[index][0][0][0]) + int(founds.corners[index][0][2][0]))//2
        y = (int(founds.corners[index][0][0][1]) + int(founds.corners[index][0][2][1]))//2

        cv2.putText(
            frame,
            str(x),
            (10, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 0),
            1
        )
        cv2.putText(
            frame,
            str(y),
            (80, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1
        )





    webcam.Update_Monitor(marked_frame)
