import pysicktim as lidar
from pyintercom import get_intercom_instance

intercom = get_intercom_instance()

while True:
    lidar.scan()
    points = [val for val in lidar.scan.distances if val > 0.01].sort()
    print(points[:10])
    intercom.publish("lidar", points[0])
