from intercom_tools import intercom
import time

def get_current_date():
    from datetime import datetime
    now = datetime.now()
    timestamp = datetime.timestamp(now)
    return {
        "date": now,
        "date_timespamp": timestamp
    }

last_print = 0
lidar_nearest_point = 0.0
odo_infos = [0.0, 0.0, 0.0]
cmd_infos = [0.0, 0.0, 0.0]

# Topic Updater
def lidar_reader(data):
    global lidar_nearest_point
    lidar_nearest_point = data


def odo_x(data):
    global odo_infos
    odo_infos[0] = data
def odo_y(data):
    global odo_infos
    odo_infos[1] = data
def odo_theta(data):
    global odo_infos
    odo_infos[2] = data


def cmd_x(data):
    global cmd_infos
    cmd_infos[0] = data
def cmd_y(data):
    global cmd_infos
    cmd_infos[1] = data
def cmd_theta(data):
    global cmd_infos
    cmd_infos[2] = data

intercom.subscribe("lidar", lidar_reader)

intercom.subscribe("x_odo", odo_x)
intercom.subscribe("y_odo", odo_y)
intercom.subscribe("theta_odo", odo_theta)

intercom.subscribe("x_cmd", cmd_x)
intercom.subscribe("y_cmd", cmd_y)
intercom.subscribe("theta_cmd", cmd_theta)
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
              f"THETA: {cmd_infos[2]}\n\n")

    # Urgency Stop
    if lidar_nearest_point < 0.1:
        intercom.publish_event("stop")