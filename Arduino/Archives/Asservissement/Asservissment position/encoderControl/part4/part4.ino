#include <util/atomic.h> 
// Motor 1
#define M1_ENCA 18
#define M1_ENCB 19
#define M1_PWM 5
#define M1_IN2 6
#define M1_IN1 7

// Motor 2
#define M2_ENCA 20
#define M2_ENCB 21
#define M2_PWM 10
#define M2_IN2 11
#define M2_IN1 12

long prevT = 0;

volatile int M1_posi = 0; 
float M1_eprev = 0;
float M1_eintegral = 0;

volatile int M2_posi = 0; 
float M2_eprev = 0;
float M2_eintegral = 0;

void setup() {
  Serial.begin(9600);
  pinMode(M1_ENCA,INPUT);
  pinMode(M1_ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(M1_ENCA), M1_readEncoder,RISING);
  
  pinMode(M1_PWM,OUTPUT);
  pinMode(M1_IN1,OUTPUT);
  pinMode(M1_IN2,OUTPUT);

  pinMode(M2_ENCA,INPUT);
  pinMode(M2_ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(M2_ENCA), M2_readEncoder,RISING);
  
  pinMode(M2_PWM,OUTPUT);
  pinMode(M2_IN1,OUTPUT);
  pinMode(M2_IN2,OUTPUT);
  
  Serial.println("M1_target M1_pos M2_target M2_pos");
}

void loop() {

  // set target position
  //target = analogRead(A0);
  //int target = 500*sin(prevT/1e6);
  int target = 0;
  int M1_target = target;
  int M2_target = target;

  // PID constants
  float kp = 1;
  float kd = 0.025;
  float ki = 3.0;
  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  int M1_pos = 0; 
  int M2_pos = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    M1_pos = M1_posi;
    M2_pos = M2_posi;
  }
  
  // error
  int M1_e = M1_pos - M1_target;
  int M2_e = M2_pos - M2_target;

  // derivative
  float M1_dedt = (M1_e-M1_eprev)/(deltaT);
  float M2_dedt = (M2_e-M2_eprev)/(deltaT);

  // integral
  M1_eintegral = M1_eintegral + M1_e*deltaT;
  M2_eintegral = M2_eintegral + M2_e*deltaT;

  // control signal
  float M1_u = kp*M1_e + kd*M1_dedt + ki*M1_eintegral;
  float M2_u = kp*M2_e + kd*M2_dedt + ki*M2_eintegral;

  // motor power
  float M1_pwr = fabs(M1_u);
  if( M1_pwr > 255 ){
    M1_pwr = 255;
  }

  float M2_pwr = fabs(M2_u);
  if( M2_pwr > 255 ){
    M2_pwr = 255;
  }
  
  // motor direction
  int M1_dir = 1;
  if(M1_u<0){
    M1_dir = -1;
  }

  int M2_dir = 1;
  if(M2_u<0){
    M2_dir = -1;
  }
  
  // signal the motor
  setMotor(M1_dir, M1_pwr, M1_PWM, M1_IN1, M1_IN2);
  setMotor(M2_dir, M2_pwr, M2_PWM, M2_IN1, M2_IN2);

  // store previous error
  M1_eprev = M1_e;
  M2_eprev = M2_e;

  Serial.print(M1_target);
  Serial.print(" ");
  Serial.print(M1_pos);
  Serial.print(" ");
  
  Serial.print(M2_target);
  Serial.print(" ");
  Serial.print(M2_pos);

  Serial.println();
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void M1_readEncoder(){
  int b = digitalRead(M1_ENCB);
  if(b > 0){
    M1_posi++;
  }
  else{
    M1_posi--;
  }
}

void M2_readEncoder(){
  int b = digitalRead(M2_ENCB);
  if(b > 0){
    M2_posi++;
  }
  else{
    M2_posi--;
  }
}
