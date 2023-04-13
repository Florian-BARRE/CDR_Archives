# Load undistort coef and try to undistort image
from classes.camera import Camera
from classes.calibration_camera.chess_calibration_camera import Camera_Calibrator
from classes.arucoRecognizer import ArucoRecognizer, draw_markers, draw_corners, draw_barycenter, extract_ellipse_info
import os
import cv2
import numpy as np
import math
from pyintercom import get_intercom_instance

intercom = get_intercom_instance()

aruco_ia = ArucoRecognizer("DICT_4X4_100")
webcam = Camera()
path = os.path.join("classes", "calibration_camera", "distortion_coef", "raspicam_undistort_coef.json")
cam_calibrator = Camera_Calibrator("", (0, 0))
cam_calibrator.load_undistort_coef(path)
hauteur_cam = 20
def midpoint(ptA, ptB):
    return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)

def fonction_de_tendance(taille_cote):
    return 6323.6 * math.pow(taille_cote, -1.018)
def distance_between_two_points(a, b):
    return math.sqrt(
            math.pow(a[0] - b[0], 2) +
            math.pow(a[1] - b[1], 2)
        )

while True:
    before_frame = webcam.Read()
    after_frame = cam_calibrator.undistort_image(before_frame)

    founds = aruco_ia.detect(after_frame)
    draw_markers(after_frame, founds)
    draw_barycenter(after_frame, founds)

    radius_center = extract_ellipse_info(founds)
    #print(radius_center)

    palets_pts = []
    for index in range(founds.len):
        A_coords = (founds.corners[index][0][0][0], founds.corners[0][0][0][1])
        B_coords = (founds.corners[index][0][1][0], founds.corners[0][0][1][1])
        C_coords = (founds.corners[index][0][2][0], founds.corners[0][0][2][1])
        D_coords = (founds.corners[index][0][3][0], founds.corners[0][0][3][1])

        bary = ((A_coords[0] + C_coords[0]) // 2, (A_coords[1] + C_coords[1]) // 2)

        distance_camera_code = fonction_de_tendance(radius_center[index]["max_radius"])
        coef_directeur_origin_bary = (bary[1] - after_frame.shape[0]) / (bary[0] - after_frame.shape[1] / 2)
        theta = math.atan(abs(coef_directeur_origin_bary))
        try:
            distance_focal_camera_code = math.sqrt(math.pow(distance_camera_code, 2) - math.pow(hauteur_cam, 2))
        except:
            distance_focal_camera_code = -1

        dx_camera_code_sol = distance_focal_camera_code * math.cos(theta)
        dy_camera_code_sol = distance_focal_camera_code * math.sin(theta)
        palets_pts.append((dx_camera_code_sol, dy_camera_code_sol))

    cv2.imwrite(os.path.join("captures", "BEFORE.jpg"), before_frame)
    cv2.imwrite(os.path.join("captures", "AFTER.jpg"), after_frame)

    intercom.publish("markers", palets_pts)


