from pyintercom import get_intercom_instance

intercom = get_intercom_instance()

def test_topic(data):
    print("message reçu :", data)

intercom.subscribe('test', test_topic)


intercom.wait_here()