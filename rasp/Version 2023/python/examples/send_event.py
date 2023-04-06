from pyintercom import get_intercom_instance


intercom = get_intercom_instance()

input("Press enter to stop the other processes...")
intercom.publish_event("exit")
