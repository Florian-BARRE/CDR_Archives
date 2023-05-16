import numpy as np
import cv2

if __name__ == '__main__':
    # termination criteria TODO: a determiner son role
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # Create ref points (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0) -> 3D
    # Choose chess size
    chess_gameboard_line_number = 12
    chess_gameboard_col_number  = 18

    ref_points = np.zeros(
        (chess_gameboard_line_number*chess_gameboard_col_number, 3),
        np.float32
    )
    # Associate each point to a grid position
    ref_points[:, :2] = np.mgrid[0:chess_gameboard_line_number, 0:chess_gameboard_col_number].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    frame = cv2.imread("./calibration_pictures/cal2_sol.jpg")
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    is_chess_on_picture, corners = cv2.findChessboardCorners(gray, (chess_gameboard_line_number, chess_gameboard_col_number), None)

    # If found, add object points, image points (after refining them)
    if is_chess_on_picture:
        objpoints.append(ref_points)

        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        imgpoints.append(corners2)
        # Draw and display the corners
        cv2.drawChessboardCorners(
            frame,
            (chess_gameboard_line_number, chess_gameboard_col_number),
            corners2,
            is_chess_on_picture
        )

        cv2.imshow('img', frame)
        cv2.waitKey(1000)

        # Camera calibration
        # return -> ret | camera matrix | distortion coefficients | rotation | translation vectors
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        # get frame width and height
        h, w = frame.shape[:2]
        # Get the optimal camera matrix (remove pixel where the image will be not linear scale)
        # roi is the best cropped to apply if we want a no deformed picture (without black pixel)

        #If the scaling parameter alpha=0, it returns undistorted image with minimum unwanted pixels.
        # So it may even remove some pixels at image corners.
        # If alpha=1, all pixels are retained with some extra black images.
        alpha = 1
        new_camera_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), alpha, (w, h))

        # undistort
        dst = cv2.undistort(frame, mtx, dist, None, new_camera_mtx)
        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        cv2.imshow('img rect', dst)
        cv2.waitKey(1000)

cv2.destroyAllWindows()
