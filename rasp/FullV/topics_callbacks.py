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

def update_markers(data):
    global markers
    markers = data