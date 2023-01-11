import math
import cv2
from imutils import perspective
from scipy.spatial import distance as dist
from imutils import contours
from classes.camera import Camera
from classes.arucoRecognizer import ArucoRecognizer, draw_markers, draw_corners, draw_barycenter
from classes.calibration_cameraV2.chess_calibration_camera import Camera_Calibrator
import numpy as np
from os import path

import imutils

webcam = Camera(res_w=1280)
aruco_ia = ArucoRecognizer("DICT_4X4_100")
hauteur_cam = 20
cam_calibrator = Camera_Calibrator(path.join("classes", "calibration_cameraV2", "multi_calibration_pictures_full_size"),
                                   (7, 7))  # 12 / 18


# cam_calibrator.init_calibration_cam(save_recognize_picture_copy=True)


def midpoint(ptA, ptB):
    return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)


while True:
    frame = webcam.Read()
    # frame = cam_calibrator.undistort_image(frame)

    founds = aruco_ia.detect(frame)

    # draw_barycenter(frame, founds)
    # marked_frame = draw_markers(frame, founds)

    for index in range(founds.len):
        draw_corners(frame, founds)
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
        cote_max_moy = (AB + CD) / 2
        cote_max = cote_max_moy
        # cote_max = max(AB, BC, CD, DA)
        distance = 6323.6 * math.pow(cote_max, -1.018)

        bary_x = (int(founds.corners[index][0][0][0]) + int(founds.corners[index][0][2][0])) // 2
        bary_y = (int(founds.corners[index][0][0][1]) + int(founds.corners[index][0][2][1])) // 2

        # Calcul elipse:
        alpha = abs(bary_y - founds.corners[0][0][0][1])
        beta = abs(bary_x - founds.corners[0][0][1][0])

        # f(x, y) = racine( (alpha.y))
        def f(x, y):
            return math.sqrt(math.pow((alpha*y), 2) + math.pow((beta*x), 2))


        cv2.putText(
            frame,
            str(theta_elipse),
            (500, 200),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 0),
            1
        )


        # X line: (w/2, 0) -> (w/2, y)
        cv2.line(
            frame,
            (int(frame.shape[1] / 2), bary_y),
            (bary_x, bary_y),
            (0, 255, 0),
            2
        )
        cv2.line(
            frame,
            (int(frame.shape[1] / 2), frame.shape[0]),
            (bary_x, bary_y),
            (255, 0, 0),
            2
        )

        dB_m = (bary_y - frame.shape[0]) / (bary_x - int(frame.shape[1] / 2))

        # Calcul angle
        # teta = math.atan(abs((dA_m - dB_m) / (1 + dA_m*dB_m)))
        teta = math.atan(abs(dB_m))

        # Distance focale sol
        df_sol = math.sqrt(math.pow(distance, 2) - math.pow(hauteur_cam, 2))
        reel_x = df_sol * math.cos(teta)
        reel_y = df_sol * math.sin(teta)

        cv2.putText(
            frame,
            str(teta),
            (10, 200),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 0),
            1
        )
        cv2.putText(
            frame,
            str(df_sol),
            (10, 100),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            1
        )
        cv2.putText(
            frame,
            str(reel_x),
            (10, 140),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            1
        )
        cv2.putText(
            frame,
            str(reel_y),
            (10, 180),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            1
        )

        cv2.putText(
            frame,
            str(distance),
            (10, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 255),
            1
        )

    webcam.Update_Monitor(frame)
