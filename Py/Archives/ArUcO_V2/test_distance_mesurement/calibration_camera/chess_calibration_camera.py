import cv2
import numpy as np


class Camera_Calibrator:
    # Criteria for chess gameboard recognize
    _chess_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    def __init__(self, cal_picture_path: str, chess_size: tuple[int, int]):
        # Init parameters variables

        self._picture_path = cal_picture_path
        # (h, w) <=> (lines, cols)
        self._chess_size = chess_size

        # Init variables

        # Create ref points (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0) -> 3D
        # They are the theoretic real world points with which we will compare image's points.
        self._theoretic_pts = [
            np.zeros(
                (self._chess_size[0] * self._chess_size[1], 3),
                np.float32
            )
        ]  # 3d points in real world space
        # Associate each point to a grid position
        self._theoretic_pts[0][:, :2] = np.mgrid[0:self._chess_size[0], 0:self._chess_size[1]].T.reshape(-1, 2)

        # Init array which will store image's points
        self._image_pts = []  # 2d points in image plane.

        # Load the calibration picture
        self._cal_frame = cv2.imread(self._picture_path)
        self._gray_cal_frame = cv2.cvtColor(self._cal_frame, cv2.COLOR_BGR2GRAY)

        # Get cal picture shape
        self._cal_frame_height, self._cal_frame_width = self._cal_frame.shape[:2]

    def _chess_game_board_recognize(self) -> bool:
        # Analyse cal frame and try to find a chess board
        # Find the chess board corners
        finding_success, corners = cv2.findChessboardCorners(
            self._gray_cal_frame,
            (self._chess_size[0], self._chess_size[1]),
            None
        )

        # Continue only if a chess board was found
        if finding_success:
            # Refine the result to be more precise
            precise_corners = cv2.cornerSubPix(
                self._gray_cal_frame,
                corners,
                (11, 11),
                (-1, -1),
                self._chess_criteria
            )

            # Add the image's point to its storage location
            self._image_pts.append(precise_corners)


            # Draw and display the corners
            """
            cv2.drawChessboardCorners(
                self._cal_frame,
                (self._chess_size[0], self._chess_size[1]),
                precise_corners,
                True
            )
            cv2.imwrite("A.jpg", self._cal_frame)
            """


        return finding_success

    def _calculate_undistorted_coef(self, alpha=1):
        # Do the camera calibration help with image's points and theoretics points
        # return -> ret | camera matrix | distortion coefficients | rotation | translation vectors
        _, self._cam_matrix, self._distortion_coef, rotation_vector, translation_vector = cv2.calibrateCamera(
            self._theoretic_pts,
            self._image_pts,
            self._gray_cal_frame.shape[::-1],
            None, None
        )

        # Get the optimal camera matrix (remove pixel where the image will be not linear scale)
        # roi is the best cropped to apply if we want a no deformed picture (without black pixel)
        # If the scaling parameter alpha=0, it returns undistorted image with minimum unwanted pixels.
        # So it may even remove some pixels at image corners.
        # If alpha=1, all pixels are retained with some extra black images.
        self._optimized_cam_matrix, self._roi = cv2.getOptimalNewCameraMatrix(
            self._cam_matrix,
            self._distortion_coef,
            (self._cal_frame_width, self._cal_frame_height),
            alpha,
            (self._cal_frame_width, self._cal_frame_height)
        )

    def init_calibration_cam(self) -> bool:
        success = self._chess_game_board_recognize()
        if success:
            self._calculate_undistorted_coef()
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
