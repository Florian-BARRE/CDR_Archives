#include "lib.h"

// Motor Left
#define L_ENCA 18
#define L_ENCB 19
#define L_PWM 5
#define L_IN2 6
#define L_IN1 7

// Motor 2
#define R_ENCA 20
#define R_ENCB 21
#define R_PWM 10
#define R_IN2 11
#define R_IN1 12

/* Régler le PID
 *  kp -> proportionnel
 *  /-> Permet de régler la vitesse de réaction du système 
 *  ki -> intégral
 *  /-> Permet de réduire l'erreur statique (en régime stationnaire)
 *  kd -> dérivation
 *  /-> Permet d'améliorer la stabilité du système
 *  
 *  Ordre réglage :
 *  I)  Kp
 *  II) Ki
 *  III)Kd
 */
 
float kp = 1;
float kd = 0.055;
float ki = 3.0;
  
Motor left_motor( L_IN1, L_IN2, L_PWM, L_ENCA, L_ENCB, kp, kd, ki);
Motor right_motor(R_IN1, R_IN2, R_PWM, R_ENCA, R_ENCB, kp, kd, ki);

long prevT = 0;

void setup() {
  Serial.begin(9600);
  
  left_motor.init();
  attachInterrupt(digitalPinToInterrupt(L_ENCA), left_motor_read_encoder, RISING);
  
  right_motor.init();
  attachInterrupt(digitalPinToInterrupt(R_ENCA), right_motor_read_encoder, RISING);
  
  Serial.println("M1_target M1_pos M2_target M2_pos");
}

// Attach Interrupt
void left_motor_read_encoder(){
    if(digitalRead(L_ENCB) > 0) left_motor.posi++;
    else                        left_motor.posi--;
}

void right_motor_read_encoder(){
    if(digitalRead(R_ENCB) > 0) right_motor.posi++;
    else                        right_motor.posi--;
}


void loop() {
  /*
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;
  */
  float deltaT = delta_time_calculator(prevT);

  int target = 500*sin(2*prevT/1e6);

  left_motor.handlee(deltaT , target, 100);
  right_motor.handlee(deltaT, target, 100);
  Serial.println();
}

float delta_time_calculator(long &previous_time){
  long current_time = micros();
  float delta_time = ((float) (current_time - previous_time))/( 1.0e6 );
  previous_time = current_time;
  return delta_time;
}
