from pyintercom import get_intercom_instance
import time

intercom = get_intercom_instance()

while True:
    intercom.publish("stop", "a")
    intercom.publish_event("send_teesny")
    print("hey")
    time.sleep(1)




