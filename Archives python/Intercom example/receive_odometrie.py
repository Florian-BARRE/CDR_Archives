from pyintercom import get_intercom_instance

def get_current_date():
    from datetime import datetime
    now = datetime.now()
    timestamp = datetime.timestamp(now)
    return {
        "date": now,
        "date_timespamp": timestamp
    }

intercom = get_intercom_instance()

o_x = 0.0
o_y = 0.0
o_theta = 0.0

c_x, c_y, c_theta = 0.0, 0.0, 0.0

def odo_x(data):
    global o_x
    o_x = data


def odo_y(data):
    global o_y
    o_y = data


def odo_theta(data):
    global o_theta
    o_theta = data


def cmd_x(data):
    global c_x
    c_x = data

def cmd_y(data):
    global c_y
    c_y = data

def cmd_theta(data):
    global c_theta
    c_theta = data

intercom.subscribe("x_odo", odo_x)
intercom.subscribe("y_odo", odo_y)
intercom.subscribe("theta_odo", odo_theta)

intercom.subscribe("x_cmd", cmd_x)
intercom.subscribe("y_cmd", cmd_y)
intercom.subscribe("theta_cmd", cmd_theta)

last_print = 0
while True:
    intercom.run_callbacks()

    if((get_current_date()["date_timespamp"] - last_print) > 0.5):
        last_print = get_current_date()["date_timespamp"]
        print(f"#-- ODOMETRIE --#\n"
              f"X: {o_x}\n"
              f"Y: {o_y}\n"
              f"THETA: {o_theta}\n"
              f"\n"
              f"#-- CMD --#\n"
              f"X: {c_x}\n"
              f"Y: {c_y}\n"
              f"THETA: {c_theta}\n\n")

