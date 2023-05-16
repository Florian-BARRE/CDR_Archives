# Just run camera and capture camera view
# Save a picture every 10s
from classes.camera import Camera
from time import sleep

webcam = Camera()

while True:
    frame = webcam.Read()
    webcam.Save("captures")
    print("New capture !")
    sleep(10)

