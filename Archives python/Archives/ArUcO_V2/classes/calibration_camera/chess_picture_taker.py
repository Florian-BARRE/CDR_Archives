from datetime import datetime
from cv2 import imwrite
from classes.camera import Camera

# Take a picture after 10s
if __name__ == '__main__':
    webcam = Camera()
    frame = webcam.Read()

    delay = 10

    date_now = datetime.now().timestamp()

    while datetime.now().timestamp() - date_now < 10 :
        print(f"Countdown: {10 - (datetime.now().timestamp() - date_now)}")
        frame = webcam.Read()
        webcam.Update_Monitor(frame)

    picture_name = input("Enter the picture name:\n")
    imwrite(f"./calibration_pictures/{picture_name}.jpg", frame)
