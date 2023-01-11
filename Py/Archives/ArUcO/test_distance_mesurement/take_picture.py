import cv2
from classes.camera import Camera

webcam = Camera()

for index in range(0):
    frame = webcam.Read()

    cv2.imwrite(f"./photos_distortion/{index}.jpg", frame)


frame = webcam.Read()
cv2.imwrite(f"./photos_distortion/cal.jpg", frame)

