#include "motors_lib.h"

void setup() {
  Serial.begin(2000000);
  motors_init();
}

void loop() {
  
    Forward_r(10);
    Backward_l(10); 
    delay(2000);
    Forward_l(10);
    Backward_r(10); 
    delay(2000);
    Stop();
    delay(4000); 
  
}
