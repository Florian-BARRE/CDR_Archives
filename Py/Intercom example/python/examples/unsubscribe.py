from pyintercom import get_intercom_instance
import time


def recv(data):
    print("received:", data)


intercom = get_intercom_instance()
ref_a = intercom.subscribe("topic", recv)
ref_b = intercom.subscribe("topic", recv)

intercom.wait_in_new_thread()

print("Sending to both callbacks...")
intercom.publish("topic", "received twice")
intercom.run_callbacks()

time.sleep(1)

print("Unsubscribing...")
intercom.unsubscribe(ref_a)
intercom.publish("topic", "received only once")
intercom.run_callbacks()

time.sleep(1)
