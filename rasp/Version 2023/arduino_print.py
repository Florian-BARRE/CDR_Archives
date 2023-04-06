from pyintercom import get_intercom_instance

intercom = get_intercom_instance()

def serialPrint(data):
    print(data)

intercom.subscribe("print", serialPrint)

intercom.wait_here()