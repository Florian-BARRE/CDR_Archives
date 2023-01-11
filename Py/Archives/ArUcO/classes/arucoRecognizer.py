import cv2
import numpy as np


class FrameDetection:
    def __init__(self, ids: list[int], corners: list[np.array]):
        self.ids = ids
        self.corners = corners
        self.len = 0 if ids is None else len(ids)


class ArucoRecognizer:

    def __init__(self, dict_type: str):
        self._aruco_dict = cv2.aruco.Dictionary_get(getattr(cv2.aruco, dict_type))
        self._aruco_params = cv2.aruco.DetectorParameters_create()

    def detect(self, frame: np.ndarray, **extra_params) -> FrameDetection:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_founded = cv2.aruco.detectMarkers(gray, self._aruco_dict, parameters=self._aruco_params, **extra_params)
        return FrameDetection(aruco_founded[1], aruco_founded[0])


def draw_markers(frame: np.ndarray, detectobj: FrameDetection) -> np.ndarray:
    if detectobj.len:
        frame = cv2.aruco.drawDetectedMarkers(frame, detectobj.corners, detectobj.ids)
    return frame


def _draw_circle_with_label(frame: np.ndarray, x: int, y: int, label: str, radius: int, thickness: int, color: tuple[int, int, int]) -> None:
        cv2.circle(
            frame,
            (x, y),
            radius,
            color,
            thickness
        )

        cv2.putText(
            frame,
            label,
            (x+10, y+10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            color,
            2
        )

def draw_corners(frame: np.ndarray, detectobj: FrameDetection, radius=5, thickness=2) -> np.ndarray:

    # Corners deco
    corners_deco = [
        {"label": "A", "color": (255, 255, 0  )},
        {"label": "B", "color": (0  , 255, 255)},
        {"label": "C", "color": (255, 0  , 255)},
        {"label": "D", "color": (255, 255, 255)},
    ]
    for index in range(detectobj.len):
        for deco_index in range(len(corners_deco)):
            _draw_circle_with_label(
                frame,
                int(detectobj.corners[index][0][deco_index][0]),
                int(detectobj.corners[index][0][deco_index][1]),
                corners_deco[deco_index]["label"],
                radius,
                thickness,
                corners_deco[deco_index]["color"]
            )

    return frame


def draw_barycenter(frame: np.ndarray, detectobj: FrameDetection, radius=5, thickness=2, color=(255, 0, 0)) -> np.ndarray:
    for index in range(detectobj.len):
        cv2.line(
            frame,
            (int(detectobj.corners[index][0][0][0]), int(detectobj.corners[index][0][0][1])),
            (int(detectobj.corners[index][0][2][0]), int(detectobj.corners[index][0][2][1])),
            color,
            thickness
        )
        cv2.line(
            frame,
            (int(detectobj.corners[index][0][1][0]), int(detectobj.corners[index][0][1][1])),
            (int(detectobj.corners[index][0][3][0]), int(detectobj.corners[index][0][3][1])),
            color,
            thickness
        )

        # Calcul Xcenter
        x = (int(detectobj.corners[index][0][0][0]) + int(detectobj.corners[index][0][2][0]))//2
        y = (int(detectobj.corners[index][0][0][1]) + int(detectobj.corners[index][0][2][1]))//2

        _draw_circle_with_label(
            frame,
            x,
            y,
            f"X: {x}  Y:{y}",
            radius,
            thickness,
            color
        )

    return frame

