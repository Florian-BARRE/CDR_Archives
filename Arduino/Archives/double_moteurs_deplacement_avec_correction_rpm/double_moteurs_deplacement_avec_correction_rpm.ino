/*
 * LEFT
 * Moteur A:
 * Sens F -> Pin 2
 * Sens B -> Pin 4
 * PWM    -> Pin 3
 * Tick   -> Pin 5
 * 
 * RIGHT
 * Moteur B:
 * Sens F -> Pin 7
 * Sens B -> Pin 8
 * PWM    -> Pin 9
 * Tick   -> Pin 10
 * 
 */
#include "motor_control.h"

void setup() {
  Serial.begin(2000000);
  motors_init();
}


void loop() {
  //forward_rpm(100.0, 8, 5);
}
