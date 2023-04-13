from pyintercom import get_intercom_instance
from time import sleep

intercom = get_intercom_instance()


def send_data_with_return(send_topic, receive_topic, data):
    global device_value
    device_value = "default"
    def update_value(data):
        global device_value
        device_value = data

    intercom.subscribe(receive_topic, update_value)
    
    while str(device_value) != str(data):
        print(f"{send_topic} {data} {device_value}")
        intercom.publish(send_topic, data)
        intercom.run_callbacks()
        #sleep(0.5)
        
    #print(f"{data} {device_value} {receive_topic}")
    #intercom.unsubscribe(receive_topic) ne fonctionne pas

def simple_send_data(send_topic, data):
    intercom.publish(send_topic, data)
    intercom.run_callbacks()

def send_coords(target_x, target_y, target_theta = 1234.12):
    send_data_with_return("x", "x_cmd", target_x)
    send_data_with_return("y", "y_cmd", target_y)
    send_data_with_return("theta", "theta_cmd", target_theta)

def update_target_point():
    intercom.publish_event("new_point")