from pyintercom import get_intercom_instance
from gpiozero import Button
from time import sleep
intercom = get_intercom_instance()

Tirette = None

Tirette =  Button(pin = 26,pull_up=True)
    
while True:
    print(Tirette.value)
    intercom.publish("tirette", Tirette.value)
    sleep(0.1)