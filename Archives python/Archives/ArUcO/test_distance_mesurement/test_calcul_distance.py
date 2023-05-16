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

    if founds.len:
        AB = math.sqrt(
            math.pow(founds.corners[0][0][0][0] - founds.corners[0][0][1][0], 2) +
            math.pow(founds.corners[0][0][0][1] - founds.corners[0][0][1][1], 2)
        )
        BC = math.sqrt(
            math.pow(founds.corners[0][0][1][0] - founds.corners[0][0][2][0], 2) +
            math.pow(founds.corners[0][0][1][1] - founds.corners[0][0][2][1], 2)
        )
        CD = math.sqrt(
            math.pow(founds.corners[0][0][2][0] - founds.corners[0][0][3][0], 2) +
            math.pow(founds.corners[0][0][2][1] - founds.corners[0][0][3][1], 2)
        )
        DA = math.sqrt(
            math.pow(founds.corners[0][0][3][0] - founds.corners[0][0][0][0], 2) +
            math.pow(founds.corners[0][0][3][1] - founds.corners[0][0][0][1], 2)
        )
        cote_max = max(AB, BC, CD, DA)

        cv2.putText(
            frame,
            str(cote_max),
            (10, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 0),
            1
        )

        #cote_distance_reel = 5
        #dx_cam_code = -1 * cote_max + 129
        a = -(46 / 34)
        b = 99 - 36 * a
        dx_cam_code = a * cote_max + b

        cv2.putText(
            frame,
            str(dx_cam_code),
            (10, 80),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1
        )




    webcam.Update_Monitor(marked_frame)
