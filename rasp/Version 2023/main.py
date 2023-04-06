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


while True:
    intercom.subscribe("lidar", lidar_reader)

    intercom.subscribe("odo_x", odo_x)
    intercom.subscribe("odo_y", odo_y)
    intercom.subscribe("odo_theta", odo_theta)

    intercom.subscribe("cmd_x", cmd_x)
    intercom.subscribe("cmd_y", cmd_y)
    intercom.subscribe("cmd_theta", cmd_theta)

    # Print
    if ((get_current_date()["date_timespamp"] - last_print) > 0.5):
        last_print = get_current_date()["date_timespamp"]
        print(f"#-- ODOMETRIE --#\n"
              f"X: {odo_infos[0]}\n"
              f"Y: {odo_infos[1]}\n"
              f"THETA: {odo_infos[2]}\n"
              f"\n"
              f"#-- CMD --#\n"
              f"X: {cmd_infos[0]}\n"
              f"Y: {cmd_infos[1]}\n"
              f"THETA: {cmd_infos[2]}\n\n")

    # Urgency Stop
    if lidar_nearest_point < 0.02:
        intercom.publish("stop")