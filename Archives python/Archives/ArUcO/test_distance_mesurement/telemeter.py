# install opencv "pip install opencv-python"
import cv2
import math
from ..classes.camera import Camera
from ..classes.arucoRecognizer import ArucoRecognizer, draw_markers, draw_corners, draw_barycenter

webcam = Camera()
aruco_ia = ArucoRecognizer("DICT_4X4_100")

# distance from camera to object(face) measured
# centimeter
Known_distance = 66.8

# width of face in the real world or Object Plane
# centimeter
Known_width = 4.9

# Colors
GREEN = (0, 255, 0)
RED = (0, 0, 255)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

# defining the fonts
fonts = cv2.FONT_HERSHEY_COMPLEX

# focal length finder function
def Focal_Length_Finder(measured_distance, real_width, width_in_rf_image):
    # finding the focal length
    focal_length = (width_in_rf_image * measured_distance) / real_width
    return focal_length


# distance estimation function
def Distance_finder(Focal_Length, real_face_width, face_width_in_frame):
    distance = (real_face_width * Focal_Length) / face_width_in_frame

    # return the distance
    return distance


def face_data(image):
    cote_max = 0  # making face width to zero

    # detecting face in the image
    founds = aruco_ia.detect(image)

    if founds.len:
        AB = math.sqrt(
            math.pow(founds.corners[0][0][0][0] - founds.corners[0][0][1][0], 2) +
            math.pow(founds.corners[0][0][0][1] - founds.corners[0][0][1][1], 2)
        )
        BC = math.sqrt(
            math.pow(founds.corners[0][0][1][0] - founds.corners[0][0][2][0], 2) +
            math.pow(founds.corners[0][0][1][1] - founds.corners[0][0][2][1], 2)
        )
        CD = math.sqrt(
            math.pow(founds.corners[0][0][2][0] - founds.corners[0][0][3][0], 2) +
            math.pow(founds.corners[0][0][2][1] - founds.corners[0][0][3][1], 2)
        )
        DA = math.sqrt(
            math.pow(founds.corners[0][0][3][0] - founds.corners[0][0][0][0], 2) +
            math.pow(founds.corners[0][0][3][1] - founds.corners[0][0][0][1], 2)
        )
        cote_max = max(AB, BC, CD, DA)

    # return the face width in pixel
    return cote_max


# reading reference_image from directory
ref_image = cv2.imread("palet_ref.jpg")

# find the face width(pixels) in the reference_image
ref_image_face_width = face_data(ref_image)

# get the focal by calling "Focal_Length_Finder"
# face width in reference(pixels),
# Known_distance(centimeters),
# known_width(centimeters)
Focal_length_found = Focal_Length_Finder(
    Known_distance, Known_width, ref_image_face_width)

print(Focal_length_found)

# show the reference image
cv2.imshow("ref_image", ref_image)

# initialize the camera object so that we
# can get frame from it

# looping through frame, incoming from
# camera/video
while True:

    # reading the frame from camera
    frame = webcam.Read()

    # calling face_data function to find
    # the width of face(pixels) in the frame
    face_width_in_frame = face_data(frame)

    # check if the face is zero then not
    # find the distance
    if face_width_in_frame != 0:
        # finding the distance by calling function
        # Distance finder function need
        # these arguments the Focal_Length,
        # Known_width(centimeters),
        # and Known_distance(centimeters)
        Distance = Distance_finder(
            Focal_length_found, Known_width, face_width_in_frame)

        # draw line as background of text
        cv2.line(frame, (30, 30), (230, 30), RED, 32)
        cv2.line(frame, (30, 30), (230, 30), BLACK, 28)

        # Drawing Text on the screen
        cv2.putText(
            frame, f"Distance: {round(Distance, 2)} CM", (30, 35),
            fonts, 0.6, GREEN, 2)

    # show the frame on the screen
    cv2.imshow("frame", frame)

    # quit the program if you press 'q' on keyboard
    if cv2.waitKey(1) == ord("q"):
        break

# closing the camera
cap.release()

# closing the windows that are opened
cv2.destroyAllWindows()
