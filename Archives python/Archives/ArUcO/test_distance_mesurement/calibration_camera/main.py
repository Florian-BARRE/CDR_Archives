if __name__ == '__main__':
    import cv2
    from classes.camera import Camera
    from chess_calibration_camera import Camera_Calibrator

    webcam = Camera()
    cam_calibrator = Camera_Calibrator("./calibration_pictures/cal1.jpg", (12, 18))

    if cam_calibrator.init_calibration_cam():
        before_frame = webcam.Read()
        after_frame = cam_calibrator.undistort_image(before_frame)
        # Helo tos
        cv2.imshow("BEFORE", before_frame)
        cv2.imshow("AFTER", after_frame)
        cv2.waitKey(100000000)
