#include "base_roulante.h"
Base_Roulante base = Base_Roulante();



void setup(){
  Serial.begin(9600);
  base.add_motor(8, 7 , 5, 2);
  base.add_motor(9, 10, 6, 3);
  base.init();
}
void loop(){
  base.forward(50);
  delay(500);
  base.forward(150);
  delay(500);
  base.forward(200);
  delay(500);
  base.forward(150);
  delay(500);
}
