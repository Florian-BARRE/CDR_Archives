from pyintercom import get_intercom_instance
import time

intercom = get_intercom_instance()

while True:
    intercom.publish("test", "message_test")
    time.sleep(1)




