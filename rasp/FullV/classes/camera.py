from time import sleep
import datetime
import cv2
import imutils
from imutils.video import VideoStream
from os import path


class Camera:
    def __init__(self, res_w=800, res_h=800):
        self.resolution = [res_h, res_w]
        self.camera = VideoStream(src=0, resolution=(self.resolution[0], self.resolution[1])).start()
        sleep(2.0)

    def Read(self):
        self.last_record_image = imutils.resize(
            self.camera.read(),
            height=self.resolution[0],
            width=self.resolution[1]
        )
        return self.last_record_image

    def Save(self, save_path, image=None):
        file_name = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")

        if image is not None:
            cv2.imwrite(path.join(save_path, f"{file_name}.jpg"), image)
        elif self.last_record_image is not None:
            cv2.imwrite(path.join(save_path, f"{file_name}.jpg"), self.last_record_image)

    def Update_Monitor(self, img, monitor_name="Frame"):
        cv2.imshow(monitor_name, img)
        cv2.waitKey(1) & 0xFF
