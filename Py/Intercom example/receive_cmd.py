from pyintercom import get_intercom_instance

intercom = get_intercom_instance()

def cmd_x(data):
    print(f"CMD x {data}")

def cmd_y(data):
    print(f"CMD y {data}")

def cmd_theta(data):
    print(f"CMD theta {data}\n")

intercom.subscribe('x', cmd_x)
intercom.subscribe('y', cmd_y)
intercom.subscribe('theta', cmd_theta)

intercom.wait_here()