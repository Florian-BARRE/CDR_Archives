#include "motors_lib.h"

Motor left_motor(8, 7 , 5, 2);
Motor right_motor(9, 10, 6, 3);

void setup(){
  Serial.begin(2000000);
  left_motor.init();
  right_motor.init();
}

void loop(){
  //right_motor.forward_RPM(100.0, 8, 5);
  //left_motor.forward_RPM(100.0, 8, 5);
  //right_motor.forward(100);
  //left_motor.forward(100);
  
  delay(1000);
  right_motor.stopp();
  left_motor.stopp();
  delay(1000);

  
  //delay(1000);
  /*
  right_motor.stopp();
  left_motor.stopp();
  delay(1000);
  right_motor.backward(100, 8, 5);
  left_motor.backward(100, 8, 5);
  delay(1000);
  
  right_motor.stopp();
  left_motor.stopp();
  delay(1000);
  */
}
