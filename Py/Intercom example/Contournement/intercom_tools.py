from pyintercom import get_intercom_instance
intercom = get_intercom_instance()


def send_data_with_return(send_topic, receive_topic, data):
    device_value = "default"
    def update_value(data):
        global device_value
        device_value = data

    intercom.subscribe(receive_topic, update_value)
    while device_value != data:
        intercom.publish(send_topic, data)
        intercom.run_callbacks()

    intercom.unsubscribe("receive_topic")

def simple_send_data(send_topic, data):
    intercom.publish(send_topic, data)
    intercom.run_callbacks()

def send_coords(tagret_x, taget_y, target_theta = 1234.1234):
    send_data_with_return("x", "x_cmd", tagret_x)
    send_data_with_return("x", "x_cmd", taget_y)
    send_data_with_return("x", "x_cmd", target_theta)

def update_target_point():
    intercom.publish_event("new_point")