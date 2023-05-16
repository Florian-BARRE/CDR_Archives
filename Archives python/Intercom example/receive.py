from pyintercom import get_intercom_instance


def stop_func(message_data):
    print("STOP: ", message_data)

def test_rev_func(message_data):
    print("TEensy meesage: ", message_data)

intercom = get_intercom_instance()
intercom.subscribe("stop", stop_func)
intercom.subscribe("send_teesny", test_rev_func)
while True:
    intercom.run_callbacks()
