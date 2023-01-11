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

// Use the "volatile" directive for variables
// used in an interrupt
// Motor 1
long M1_prevT = 0;
int M1_posPrev = 0;

volatile int M1_pos_i = 0;
volatile float M1_velocity_i = 0;
volatile long M1_prevT_i = 0;

float M1_v1Filt = 0;
float M1_v1Prev = 0;
float M1_v2Filt = 0;
float M1_v2Prev = 0;

float M1_eintegral = 0;
float M1_eprev = 0;

// Motor 2
long M2_prevT = 0;
int M2_posPrev = 0;

volatile int M2_pos_i = 0;
volatile float M2_velocity_i = 0;
volatile long M2_prevT_i = 0;

float M2_v1Filt = 0;
float M2_v1Prev = 0;
float M2_v2Filt = 0;
float M2_v2Prev = 0;

float M2_eintegral = 0;
float M2_eprev = 0;

void setup() {
  Serial.begin(115200);
  
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
  
  Serial.println("M1_target M1_reel M2_target M2_reel");
}

void loop() {
  // read the position in an atomic block
  // to avoid potential misreads
  int M1_pos = 0;
  float M1_velocity2 = 0;

  int M2_pos = 0;
  float M2_velocity2 = 0;
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    M1_pos = M1_pos_i;
    M1_velocity2 = M1_velocity_i;
    
    M2_pos = M2_pos_i;
    M2_velocity2 = M2_velocity_i;
  }

  // Compute velocity with method 1
  long M1_currT = micros();
  float M1_deltaT = ((float) (M1_currT-M1_prevT))/1.0e6;

  long M2_currT = micros();
  float M2_deltaT = ((float) (M2_currT-M2_prevT))/1.0e6;
  
  float M1_velocity1 = (M1_pos - M1_posPrev)/M1_deltaT;
  M1_posPrev = M1_pos;
  M1_prevT = M1_currT;

  float M2_velocity1 = (M2_pos - M2_posPrev)/M2_deltaT;
  M2_posPrev = M2_pos;
  M2_prevT = M2_currT;

  // Convert count/s to RPM
  float M1_v1 = M1_velocity1/600.0*60.0;
  float M1_v2 = M1_velocity2/600.0*60.0;

  float M2_v1 = M2_velocity1/600.0*60.0;
  float M2_v2 = M2_velocity2/600.0*60.0;

  // Low-pass filter (25 Hz cutoff)
  M1_v1Filt = 0.854*M1_v1Filt + 0.0728*M1_v1 + 0.0728*M1_v1Prev;
  M1_v1Prev = M1_v1;
  M1_v2Filt = 0.854*M1_v2Filt + 0.0728*M1_v2 + 0.0728*M1_v2Prev;
  M1_v2Prev = M1_v2;

  M2_v1Filt = 0.854*M2_v1Filt + 0.0728*M2_v1 + 0.0728*M2_v1Prev;
  M2_v1Prev = M2_v1;
  M2_v2Filt = 0.854*M2_v2Filt + 0.0728*M2_v2 + 0.0728*M2_v2Prev;
  M2_v2Prev = M2_v2;

  // Set a target
  // max 400 min -400
  long prevT = (M1_prevT + M2_prevT)/2;

/*
  float v_target = 50;
  if((10*sin(prevT/1e5)) < 0){
    v_target = -50;
  }*/
  
  //float v_target = 10*sin(prevT/1e5);
  float M1_vt = 10;
  float M2_vt = -10;
  if((10*sin(prevT/1e4)) < 0){
    M1_vt = -10;
    M2_vt = 10;
  }

  // Compute the control signal u
  float kp = 0.5;
  float kd = 0.0;
  float ki = 6.0;

  // error
  float M1_e = M1_vt-M1_v1Filt;
  float M2_e = M2_vt-M2_v1Filt;
  // integral
  M1_eintegral = M1_eintegral + M1_e*M1_deltaT;
  M2_eintegral = M2_eintegral + M2_e*M2_deltaT; 
  
  // derivative
  float M1_dedt = (M1_e-M1_eprev)/(M1_deltaT);
  float M2_dedt = (M2_e-M2_eprev)/(M2_deltaT);
  
  // control signal
  float M1_u = kp*M1_e + kd*M1_dedt + ki*M1_eintegral;
  float M2_u = kp*M2_e + kd*M2_dedt + ki*M2_eintegral;
  
  // Set the motor speed and direction
  int M1_dir = 1;
  if (M1_u<0){
    M1_dir = -1;
  }
  int M1_pwr = (int) fabs(M1_u);
  if(M1_pwr > 255){
    M1_pwr = 255;
  }
  
  int M2_dir = 1;
  if (M2_u<0){
    M2_dir = -1;
  }
  int M2_pwr = (int) fabs(M2_u);
  if(M2_pwr > 255){
    M2_pwr = 255;
  }
  setMotor(M1_dir, M1_pwr, M1_PWM, M1_IN1, M1_IN2);
  setMotor(M2_dir, M2_pwr, M2_PWM, M2_IN1, M2_IN2);

  // store previous error
  M1_eprev = M1_e;
  M2_eprev = M2_e;
  
  Serial.print(M1_vt);
  Serial.print(" ");
  Serial.print(M1_v1Filt);
  Serial.print(" ");
  
  Serial.print(M2_vt);
  Serial.print(" ");
  Serial.print(M2_v1Filt);

  Serial.println();
  delay(1);
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal); // Motor speed
  if(dir == 1){ 
    // Turn one way
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);    
  }
}

void M1_readEncoder(){
  // Read encoder B when ENCA rises
  int b = digitalRead(M1_ENCB);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  M1_pos_i = M1_pos_i + increment;

  // Compute velocity with method 2
  long M1_currT = micros();
  float M1_deltaT = ((float) (M1_currT - M1_prevT_i))/1.0e6;
  M1_velocity_i = increment/M1_deltaT;
  M1_prevT_i = M1_currT;
}

void M2_readEncoder(){
  // Read encoder B when ENCA rises
  int b = digitalRead(M2_ENCB);
  int increment = 0;
  if(b>0){
    // If B is high, increment forward
    increment = 1;
  }
  else{
    // Otherwise, increment backward
    increment = -1;
  }
  M2_pos_i = M2_pos_i + increment;

  // Compute velocity with method 2
  long M2_currT = micros();
  float M2_deltaT = ((float) (M2_currT - M2_prevT_i))/1.0e6;
  M2_velocity_i = increment/M2_deltaT;
  M2_prevT_i = M2_currT;
}
