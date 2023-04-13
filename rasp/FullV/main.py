from classes.intercom_tools import intercom, send_coords
from topics_callbacks import *
from classes.tools import get_current_date

last_print = 0
lidar_nearest_point = 0.0
markers = ""
odo_infos = [0.0, 0.0, 0.0]
cmd_infos = [0.0, 0.0, 0.0]

# Define the function that will be called to get around the obstacle and continue the path to the target point
def get_around_obstacle():
    print("Obstacle detected, getting around it...")
    # Get the current position of the robot
    current_x = odo_infos[0]
    current_y = odo_infos[1]

    # Get the current target point of the robot
    target_x = cmd_infos[0]
    target_y = cmd_infos[1]

    # Get the current direction of the robot
    current_theta = odo_infos[2]

    # Get the direction to the target point
    target_theta = cmd_infos[2]

    # Get the distance to the target point
    distance_to_target = ((target_x - current_x)*2 + (target_y - current_y)*2)*0.5


    # Get the direction to the obstacle
    obstacle_direction = (current_theta + 90) % 360

    # Get the distance to the obstacle
    obstacle_distance = lidar_nearest_point

    # Go around the obstacle and continue the path to the target point

    # Go 10 cm left of the obstacle
    send_coords(current_x + 0.1 * (obstacle_direction - 90) / 90, current_y + 0.1 * (obstacle_direction - 90) / 90, current_theta)

    # Or go 10 cm right of the obstacle
    send_coords(current_x + 0.1 * (obstacle_direction + 90) / 90, current_y + 0.1 * (obstacle_direction + 90) / 90, current_theta)

    # Go to the initial direction to continue the path to the target point
    # Go to the target point
    send_coords(target_x, target_y, target_theta)

intercom.subscribe("lidar", lidar_reader)

intercom.subscribe("x_odo", odo_x)
intercom.subscribe("y_odo", odo_y)
intercom.subscribe("theta_odo", odo_theta)

intercom.subscribe("x_cmd", cmd_x)
intercom.subscribe("y_cmd", cmd_y)
intercom.subscribe("theta_cmd", cmd_theta)

intercom.subscribe("markers", update_markers)

while True:
    intercom.run_callbacks()
    # Print
    if ((get_current_date()["date_timespamp"] - last_print) > 0.5):
        last_print = get_current_date()["date_timespamp"]
        print(f"#-- Lidar --#\n"
              f"Distance: {lidar_nearest_point}\n\n"
              f"#-- ODOMETRIE --#\n"
              f"X: {odo_infos[0]}\n"
              f"Y: {odo_infos[1]}\n"
              f"THETA: {odo_infos[2]}\n"
              f"\n"
              f"#-- CMD --#\n"
              f"X: {cmd_infos[0]}\n"
              f"Y: {cmd_infos[1]}\n"
              f"THETA: {cmd_infos[2]}\n"
              f"#-- Markers --#\n"
              f" {markers} \n\n")

    # Urgency Stop
    if lidar_nearest_point < 0.1:
        #intercom.publish_event("stop")
        get_around_obstacle()



