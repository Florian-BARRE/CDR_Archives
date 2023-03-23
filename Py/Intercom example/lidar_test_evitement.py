from dis import dis
import pysicktim as lidar
from math import *
from sklearn.cluster import DBSCAN
from time import sleep
from pyintercom import get_intercom_instance
import time

intercom = get_intercom_instance()

running = True

lidar.scan()
start_angle = lidar.scan.dist_start_ang % 360
angle_div = lidar.scan.dist_angle_res

thresold = 40

while running:
    lidar.scan()

    angle = start_angle
    points = []
    for val in lidar.scan.distances:
        if val > 0.01:
            points.append(val)
    points.sort()
    print(points[0:10])

    if(points[0] < 0.2):
        intercom.publish("stop", 1)
        print("TURN !")
    print()
    #sleep(1)

    """
            points.append((int(cos(radians(angle)) * val * 100), int(sin(radians(angle)) * val * 100)))

        angle = (angle - angle_div) % 360
    print(sorted(points))
   
    labels = DBSCAN(eps=5).fit_predict(points)
    objects = {}
    
    for i in range(len(points)):
        point = points[i]
        label = labels[i]

        if label in objects:
            objects[label].append(point)
        else:
            objects[label] = [point]

    front = False
    behind = False

    for object_points in objects.values():
        first_point = object_points[0]
        distance_first = sqrt(first_point[0] ** 2 + first_point[1] ** 2)

        center_point = (sum([x[0] for x in object_points]) / len(object_points), sum([x[1] for x in object_points]) / len(object_points))
        distance_center = sqrt(center_point[0] ** 2 + center_point[1] ** 2)

        last_point = object_points[-1]
        distance_last = sqrt(last_point[0] ** 2 + last_point[1] ** 2)


        if distance_first < thresold:

            display_first_point = (first_point[0] + 750, first_point[1] + 400)

            angle_first = degrees(atan2(first_point[1], first_point[0])) % 360
            if angle_first < 135:
                front = True
            elif angle_first > 225:
                behind = True

        if distance_center < thresold:

            display_center_point = (center_point[0] + 750, center_point[1] + 400)
            angle_center = degrees(atan2(center_point[1], center_point[0])) % 360
            if angle_center < 135:
                front = True
            elif angle_center > 225:
                behind = True

        if distance_last < thresold:
            display_last_point = (last_point[0] + 750, last_point[1] + 400)

            angle_last = degrees(atan2(last_point[1], last_point[0])) % 360
            if angle_last < 135:
                front = True
            elif angle_last > 225:
                behind = True
"""