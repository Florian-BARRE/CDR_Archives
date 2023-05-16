#include <Arduino.h>
#include "intercom/include/intercom.h"

int *last_received_position;
int last_received_length;


void setup() {
  Intercom::init("brain_robot_de_test", DEFAULT_INTERCOM_SPEED);
  Intercom::subscribe("test_topic");

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  Intercom::tick();

  if (Intercom::instantReceiveIntArray("test_topic", last_received_position, &last_received_length))
  {
    digitalWrite(LED_BUILTIN, 1);
    delay(1000);
    digitalWrite(LED_BUILTIN, 0);
    delay(1000);
  }
  
}