from pyintercom import get_intercom_instance
import time

intercom = get_intercom_instance()

while True:
    intercom.publish("test", 5)
    time.sleep(20)




