# Capture a picture every 10s and try to detect aruco code
from classes.camera import Camera
from classes.arucoRecognizer import ArucoRecognizer, draw_markers, draw_corners, draw_barycenter
from time import sleep

webcam = Camera()
aruco_ia = ArucoRecognizer("DICT_4X4_100")

while True:
    frame = webcam.Read()
    founds = aruco_ia.detect(frame)
    marked_frame = draw_markers(frame, founds)
    draw_corners(frame, founds)
    draw_barycenter(frame, founds)

    webcam.Save("captures")
    print("New capture !")
    sleep(10)
