import cv2
import numpy as np
import glob
from os import path, makedirs
from time import sleep


class Camera_Calibrator:
    # Criteria for chess gameboard recognize
    _chess_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    def __init__(self, cal_picture_folder_path: str, chess_size: tuple[int, int], picture_ext: str = "jpg"):
        # Init parameters variables

        self.cal_picture_folder_path = cal_picture_folder_path
        # (h, w) <=> (lines, cols)
        self._chess_size = chess_size

        # Init variables

        # Create ref points (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0) -> 3D
        # They are the theoretic real world points with which we will compare image's points.
        self._theoretic_pts_obj = np.zeros(
            (self._chess_size[0] * self._chess_size[1], 3),
            np.float32
        )
        # Associate each point to a grid position
        self._theoretic_pts_obj[:, :2] = np.mgrid[0:self._chess_size[0], 0:self._chess_size[1]].T.reshape(-1, 2)

        # Init array which will store image's points
        self._image_pts = []  # 2d points in image plane.
        # Init array which will store theoretic's points
        self._theoretic_pts = []  # 3d points in real world space

        # Load all picture's path from the folder
        self._pictures_path_list = glob.glob(f"{path.join(self.cal_picture_folder_path, '*')}.{picture_ext}")

        # Create a new path to save recognized chess pictures
        if len(self._pictures_path_list) > 0:
            path_and_file = path.split(self._pictures_path_list[0])
            self._recognized_picture_save_path = f"{path_and_file[0]}_cal_copy"
            if not path.isdir(self._recognized_picture_save_path):
                makedirs(self._recognized_picture_save_path)

    def _chess_game_board_recognize(self, gray_picture, image_path=None) -> bool:
        # Analyse cal frame and try to find a chess board
        # Find the chess board corners
        finding_success, corners = cv2.findChessboardCorners(
            gray_picture,
            (self._chess_size[0], self._chess_size[1]),
            None
        )

        # Continue only if a chess board was found
        if finding_success:
            # Refine the result to be more precise
            precise_corners = cv2.cornerSubPix(
                gray_picture,
                corners,
                (11, 11),
                (-1, -1),
                self._chess_criteria
            )

            # Add the image's point to its storage location
            self._image_pts.append(precise_corners)
            # Add the theoretic points to its storage location
            self._theoretic_pts.append(self._theoretic_pts_obj)

            # Draw and display the corners
            cv2.drawChessboardCorners(
                gray_picture,
                (self._chess_size[0], self._chess_size[1]),
                precise_corners,
                True
            )

            if image_path is not None:
                cv2.imwrite(
                    path.join(self._recognized_picture_save_path, path.split(image_path)[1]),
                    gray_picture
                )

        return finding_success

    def _calculate_undistorted_coef(self, gray_picture, alpha=1):
        # Do the camera calibration help with image's points and theoretics points
        # return -> ret | camera matrix | distortion coefficients | rotation | translation vectors
        _, self._cam_matrix, self._distortion_coef, rotation_vector, translation_vector = cv2.calibrateCamera(
            self._theoretic_pts,
            self._image_pts,
            gray_picture.shape[::-1],
            None, None
        )

        # Get the optimal camera matrix (remove pixel where the image will be not linear scale)
        # roi is the best cropped to apply if we want a no deformed picture (without black pixel)
        # If the scaling parameter alpha=0, it returns undistorted image with minimum unwanted pixels.
        # So it may even remove some pixels at image corners.
        # If alpha=1, all pixels are retained with some extra black images.
        width, height = gray_picture.shape[:2]
        self._optimized_cam_matrix, self._roi = cv2.getOptimalNewCameraMatrix(
            self._cam_matrix,
            self._distortion_coef,
            (width, height),
            alpha,
            (width, height)
        )

    def init_calibration_cam(self, save_recognize_picture_copy=False) -> bool:
        success = False
        for image in self._pictures_path_list:
            cal_frame = cv2.imread(image)
            gray_cal_frame = cv2.cvtColor(cal_frame, cv2.COLOR_BGR2GRAY)

            if save_recognize_picture_copy:
                if self._chess_game_board_recognize(gray_cal_frame, image):
                    success = True
            elif self._chess_game_board_recognize(gray_cal_frame):
                success = True

        if success:
            self._calculate_undistorted_coef(gray_cal_frame)

        return success

    def undistort_image(self, frame_to_undistort, cropped_image=False):
        # Undistort the input frame
        undistort_frame = cv2.undistort(
            frame_to_undistort,
            self._cam_matrix,
            self._distortion_coef,
            None,
            self._optimized_cam_matrix
        )

        if cropped_image:
            # crop the image
            x, y, w, h = self._roi
            undistort_frame = undistort_frame[y:y + h, x:x + w]

        return undistort_frame
