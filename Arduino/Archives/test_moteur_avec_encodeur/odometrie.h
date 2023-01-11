/*
 * Moteur A:
 * Sens F -> Pin 2
 * Sens B -> Pin 4
 * PWM    -> Pin 3
 * Tick   -> Pin 5
 * 
 * Moteur B:
 * Sens F -> Pin 7
 * Sens B -> Pin 8
 * PWM    -> Pin 9
 * Tick   -> Pin 10
 * 
 */

#include <Arduino.h>
#include <math.h>


int tikleft = 0;
int tikright = 0;
double x = 0;
double y = 0;
double teta = 0;

void interruptionleft() {
  Serial.print("Digital ");
    Serial.println(digitalRead(sens_left));
  if (digitalRead(sens_left)) {
    tikleft++;
    
  }
  else {
    tikleft--;
  }
}

void interruptionright() {
  if (!digitalRead(sens_right)) {
    tikright++;
  }
  else {
    tikright--;
  }
}

void position(double dleft, double dright, double*px, double*py, double*pteta) {
  double dcenter = (dleft + dright) / 2;
  double phi = (dright - dleft) / entraxe;
  *px = x + dcenter * cos(*pteta + phi / 2);
  *py = y + dcenter * sin(*pteta + phi / 2);
  *pteta = teta + phi;
  tikright = 0;
  tikleft = 0;
}

void Setup_Odometrie(){
  attachInterrupt(digitalPinToInterrupt(inputPin_left), interruptionleft, RISING);
  attachInterrupt(digitalPinToInterrupt(inputPin_right), interruptionright, RISING);
  //NVIC_SET_PRIORITY(IRQ_PORTA, 0);
}
