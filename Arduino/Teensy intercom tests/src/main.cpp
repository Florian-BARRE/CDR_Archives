#include <Arduino.h>
#include "intercom.h"

void BLINK()
{
  digitalWrite(LED_BUILTIN, 1);
  delay(1000);
  digitalWrite(LED_BUILTIN, 0);
  delay(1000);
}

void setup() {
  Intercom::init("test", DEFAULT_INTERCOM_SPEED);
  Intercom::subscribe("stop");

  pinMode(LED_BUILTIN, OUTPUT);
  BLINK();
}

void loop() {
  int val;
  Intercom::tick();

  

  // if (Intercom::instantHasReceivedEvent("send_teesny"))
  // {
  //   BLINK();
  // }

  if (Intercom::instantReceiveInt("stop", &val))
  {
    BLINK();
  }

}

