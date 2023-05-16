#include <Arduino.h>
#include "intercom.h"

void BLINK(int n)
{
    for(int k=0; k<n; k++){
        digitalWrite(LED_BUILTIN, 1);
        delay(1000);
        digitalWrite(LED_BUILTIN, 0);
        delay(1000);
    }
}

void setup() {
  Intercom::init("arduino_test", DEFAULT_INTERCOM_SPEED);
  Intercom::subscribe("test");

  pinMode(LED_BUILTIN, OUTPUT);
  BLINK(2);
}

void loop() {
  int val;
  Intercom::tick();
  if (Intercom::instantReceiveInt("stop", &val))
  {
    BLINK(val);
  }
}

