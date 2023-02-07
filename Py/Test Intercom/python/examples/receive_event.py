from pyintercom import get_intercom_instance
import time


intercom = get_intercom_instance()
intercom.on_event("exit", lambda: exit())

while True:
    print("Running callbacks...")
    intercom.run_callbacks()
    print("Waiting for 1 second...")
    time.sleep(1)
