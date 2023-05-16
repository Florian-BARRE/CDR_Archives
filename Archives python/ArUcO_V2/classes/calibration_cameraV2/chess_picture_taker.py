from datetime import datetime
from cv2 import imwrite
from classes.camera import Camera
from os import path, makedirs

frame_names = "cal_"
save_folder = "test_simple"
frame_ext = "jpg"

delay = 100
frequency = 1

# Take a picture 50 pictures in 50s (1 picture per second)
if __name__ == '__main__':
    if not path.isdir(save_folder):
        makedirs(save_folder)

    webcam = Camera(res_w=1280)
    frame = webcam.Read()
    frame_index = 0
    date_now = datetime.now().timestamp()

    # Wait 5s before starting to take the pictures
    while datetime.now().timestamp() - date_now < 5:
        frame = webcam.Read()
        webcam.Update_Monitor(frame)
        print(f"Countdown before start of pictures taking: {5 - (datetime.now().timestamp() - date_now)}")

    start_date = datetime.now().timestamp()
    date_now = datetime.now().timestamp()

    while datetime.now().timestamp() - start_date < delay:
        frame = webcam.Read()
        webcam.Update_Monitor(frame)

        if datetime.now().timestamp() - date_now >= frequency:
            date_now = datetime.now().timestamp()
            print(f"Countdown: {delay - (datetime.now().timestamp() - start_date)}")
            imwrite(path.join(save_folder, f"{frame_names}{frame_index}.{frame_ext}"), frame)
            frame_index += 1
