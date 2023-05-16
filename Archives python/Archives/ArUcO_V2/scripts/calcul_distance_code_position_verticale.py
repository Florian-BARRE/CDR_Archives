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
largeur_cam = 1280
aruco_ia = ArucoRecognizer("DICT_4X4_100")
hauteur_cam = 20
cam_calibrator = Camera_Calibrator(path.join("classes", "calibration_cameraV2", "test_simple"),
                                   (5, 5))  # 12 / 18


cam_calibrator.init_calibration_cam(save_recognize_picture_copy=True)


def distance_between_two_points(a, b):
    return math.sqrt(
            math.pow(a[0] - b[0], 2) +
            math.pow(a[1] - b[1], 2)
        )

def fonction_de_tendance(taille_cote):
    return 6323.6 * math.pow(taille_cote, -1.018)

while True:
    frame = webcam.Read()
    frame = cam_calibrator.undistort_image(frame)

    founds = aruco_ia.detect(frame)

    draw_barycenter(frame, founds)
    draw_corners(frame, founds)
    #draw_markers(frame, founds)

    for index in range(founds.len):
        # TODO: piste amélioration -> faire des coordonnées en float (evité les calculs avec les arrondies)
        A_coords = (founds.corners[index][0][0][0], founds.corners[0][0][0][1])
        B_coords = (founds.corners[index][0][1][0], founds.corners[0][0][1][1])
        C_coords = (founds.corners[index][0][2][0], founds.corners[0][0][2][1])
        D_coords = (founds.corners[index][0][3][0], founds.corners[0][0][3][1])
        corners_coords = [A_coords, B_coords, C_coords, D_coords]

        bary = ((A_coords[0] + C_coords[0]) // 2, (A_coords[1] + C_coords[1]) // 2)

        AB = distance_between_two_points(A_coords, B_coords)
        BC = distance_between_two_points(B_coords, C_coords)
        CD = distance_between_two_points(C_coords, D_coords)
        DA = distance_between_two_points(D_coords, A_coords)

        cote_max = max(AB, BC, CD, DA)

        distance_camera_code = fonction_de_tendance(cote_max)
        coef_directeur_origin_bary = (bary[1] - frame.shape[0]) / (bary[0] - frame.shape[1] / 2)
        theta = math.atan(abs(coef_directeur_origin_bary))
        # teta = math.atan(abs((dA_m - dB_m) / (1 + dA_m*dB_m)))
        try :
            distance_focal_camera_code = math.sqrt(math.pow(distance_camera_code, 2) - math.pow(hauteur_cam, 2))
        except:
            distance_focal_camera_code = -1

        dx_camera_code_sol = distance_focal_camera_code * math.cos(theta)
        dy_camera_code_sol = distance_focal_camera_code * math.sin(theta)

        color = (0, 255, 255)
        cv2.putText(frame, f"Taille cote max du code: {round(cote_max, 1)}px.",                            (10, 20), cv2.FONT_HERSHEY_SIMPLEX,  0.5, color, 1)
        cv2.putText(frame, f"Distance deduite camera code: {round(distance_camera_code, 1)}cm.",           (10, 40), cv2.FONT_HERSHEY_SIMPLEX,  0.5, color, 1)
        cv2.putText(frame, f"Angle entre le centre image et code: {round(theta*57.2958, 2)} degres.",      (10, 60), cv2.FONT_HERSHEY_SIMPLEX,  0.5, color, 1)
        cv2.putText(frame, f"Distance focal camera code: {round(distance_focal_camera_code, 1)}cm.",       (10, 80), cv2.FONT_HERSHEY_SIMPLEX,  0.5, color, 1)
        cv2.putText(frame, f"dX camera code: {round(dx_camera_code_sol, 1)}cm.",                           (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        cv2.putText(frame, f"dY camera code: {round(dy_camera_code_sol, 1)}cm.",                           (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Ligne entre le milieu bas de la caméra et le barycentre
        cv2.line(frame, (int(bary[0]), int(bary[1])), (frame.shape[1]//2, frame.shape[0]), (255, 0, 255), 2)
        # Ligne entre le barycentre et la ligne qui sépare la caméra en deux
        cv2.line(frame, (int(bary[0]), int(bary[1])), (frame.shape[1] // 2, int(bary[1])), (255, 0, 255), 2)
        # Ligne entre le point qui est sur la ligne du mileu et le milieu bas
        cv2.line(frame, (frame.shape[1]//2, frame.shape[0]), (frame.shape[1] // 2, int(bary[1])), (255, 0, 255), 2)

    # Ligne vertical qui sépare la caméra en deux
    cv2.line(frame, (frame.shape[1] // 2, 0), (frame.shape[1] // 2, frame.shape[0]), (0, 0, 255), 1)
    webcam.Update_Monitor(frame)
