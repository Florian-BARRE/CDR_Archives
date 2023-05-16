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
cam_calibrator = Camera_Calibrator(path.join("classes", "calibration_cameraV2", "multi_calibration_pictures_full_size"),
                                   (7, 7))  # 12 / 18


# cam_calibrator.init_calibration_cam(save_recognize_picture_copy=True)


def midpoint(ptA, ptB):
    return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)

def distance_between_two_points(a, b):
    return math.sqrt(
            math.pow(a[0] - b[0], 2) +
            math.pow(a[1] - b[1], 2)
        )
while True:
    frame = webcam.Read()
    # frame = cam_calibrator.undistort_image(frame)

    founds = aruco_ia.detect(frame)

    # draw_barycenter(frame, founds)
    # marked_frame = draw_markers(frame, founds)

    for index in range(founds.len):

        A_coords = (founds.corners[index][0][0][0], founds.corners[0][0][0][1])
        B_coords = (founds.corners[index][0][1][0], founds.corners[0][0][1][1])
        C_coords = (founds.corners[index][0][2][0], founds.corners[0][0][2][1])
        D_coords = (founds.corners[index][0][3][0], founds.corners[0][0][3][1])
        corners_coords = [A_coords, B_coords, C_coords, D_coords]

        bary = ((A_coords[0] + C_coords[0]) // 2, (A_coords[1] + C_coords[1]) // 2)

        # Calcul Fiable
        AB = distance_between_two_points(A_coords, B_coords)
        BC = distance_between_two_points(B_coords, C_coords)
        CD = distance_between_two_points(C_coords, D_coords)
        DA = distance_between_two_points(D_coords, A_coords)

        cote_max = max(AB, BC, CD, DA)
        distance = 6323.6 * math.pow(cote_max, -1.018)
        dB_m = (bary[1] - frame.shape[0]) / (bary[0] - int(frame.shape[1] / 2))
        # Calcul angle
        # teta = math.atan(abs((dA_m - dB_m) / (1 + dA_m*dB_m)))
        teta = math.atan(abs(dB_m))
        # Distance focale sol
        try :
            df_sol = math.sqrt(math.pow(distance, 2) - math.pow(hauteur_cam, 2))
        except:
            df_sol = 0
        reel_x = df_sol * math.cos(teta)
        reel_y = df_sol * math.sin(teta)

        cv2.putText(frame, str(reel_x)  , (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(frame, str(reel_y)  , (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(frame, str(distance), (10, 40) , cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # En cours de recherche -> obtenir la distance avec palet à plat
        # Get le point le plus proche du centre de l'image selon x
        centre_coords = (frame.shape[1]//2, frame.shape[0]//2)
        index_pt_central = -1
        distance_min = 0
        for i in range(len(corners_coords)):
            if abs(corners_coords[i][0] - centre_coords[0]) < distance_min or i==0:
                index_pt_central = i
                distance_min = abs(corners_coords[i][0] - centre_coords[0])

        pt_central = corners_coords[index_pt_central]
        pt_oppose_central = corners_coords[(index_pt_central - 2) % 4]

        H_coords = (pt_oppose_central[0], pt_central[1])


        cv2.line(frame, H_coords, pt_central, thickness=1, color=(255, 255, 0))
        cv2.line(frame, pt_oppose_central, pt_central, thickness=1, color=(255, 255, 0))
        cv2.line(frame, pt_oppose_central, H_coords, thickness=1, color=(255, 255, 0))

        pt_central_H_distance = distance_between_two_points(H_coords, pt_central)
        hypo = distance_between_two_points(pt_oppose_central, pt_central)
        angle_rot = math.acos(pt_central_H_distance/hypo)


        cote_avant_rotation = distance_between_two_points(pt_oppose_central, pt_central)
        cote_apres_rotation = cote_avant_rotation * math.cos(angle_rot) * math.sqrt(2)

        draw_corners(frame, founds)
        cv2.putText(frame, f"Angle rotation {round(angle_rot)}, en degre:{round(angle_rot*57.2958)}", (10, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(frame, f"avant rot: {round(cote_avant_rotation)}, apres rot:{round(cote_apres_rotation)}", (10, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(frame, f"dist reel: {round(6323.6 * math.pow(cote_apres_rotation, -1.018))}", (10, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        #cv2.putText(frame, str(6323.6 * math.pow(pt_central_H_distance, -1.018)), (10, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)


        cv2.putText(
            frame,
            "Point le plus proche du centre",
            (pt_central[0], pt_central[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 255),
            1
        )
        cv2.putText(
            frame,
            "X Point H",
            (H_coords[0], H_coords[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 255),
            1
        )
        cv2.putText(
            frame,
            "X",
            (centre_coords[0], centre_coords[1]),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 0, 0),
            1
        )
    webcam.Update_Monitor(frame)
