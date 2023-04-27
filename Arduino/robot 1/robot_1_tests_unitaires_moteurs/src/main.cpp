#include <Arduino.h>
#include <rolling_basis.h>
#include <util/atomic.h>

// Creation Rolling Basis
#define ENCODER_RESOLUTION 1024
#define CENTER_DISTANCE 27.07
#define WHEEL_DIAMETER 6.1

// Motor Left
#define L_ENCA 11
#define L_ENCB 12
#define L_PWM 5
#define L_IN2 3
#define L_IN1 4

// Motor Right
#define R_ENCA 14
#define R_ENCB 13
#define R_PWM 2
#define R_IN2 1
#define R_IN1 0

float kp = 0.9;
float ki = 0.0003;
float kd = 0.0;
byte max_pwm = 100;

Rolling_Basis *rolling_basis_ptr = new Rolling_Basis(ENCODER_RESOLUTION, CENTER_DISTANCE, WHEEL_DIAMETER, max_pwm, 30);

#define BALADE_SIZE 3
byte action_index = 0;

Action action0 = create_action(0.0, 0.0);
Action action1 = create_action(160.0, 0.0);
Action action2 = create_action(0.0, 0.0);

Action balade_model[BALADE_SIZE] = {action0, action1, action2};
Action balade[BALADE_SIZE];

/******* Attach Interrupt *******/
inline void left_motor_read_encoder()
{
  if (digitalRead(L_ENCB) > 0)
    rolling_basis_ptr->left_motor->ticks++;
  else
    rolling_basis_ptr->left_motor->ticks--;
}

inline void right_motor_read_encoder()
{
  if (digitalRead(R_ENCB) > 0)
    rolling_basis_ptr->right_motor->ticks++;
  else
    rolling_basis_ptr->right_motor->ticks--;
}

// Pin ON / OFF
byte pin_on_off = 19;

// Globales variables 
long right_ticks = 0;
long left_ticks = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(pin_on_off, INPUT);

  // motors init
  analogWriteFrequency(R_PWM, 30000); 
  analogWriteFrequency(L_PWM, 30000); 

  rolling_basis_ptr->init_right_motor(R_IN1, R_IN2, R_PWM, R_ENCA, R_ENCB, kp, kd, ki, 1.0);
  rolling_basis_ptr->init_left_motor(L_IN1, L_IN2, L_PWM, L_ENCA, L_ENCB, kp, kd, ki, 1.2);
  rolling_basis_ptr->init_motors();
  attachInterrupt(digitalPinToInterrupt(L_ENCA), left_motor_read_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENCA), right_motor_read_encoder, RISING);

  // Init balade
  memcpy(balade, balade_model, sizeof(Action) * BALADE_SIZE);
}

void loop()
{

  rolling_basis_ptr->right_motor->set_motor(1, 200);
  rolling_basis_ptr->left_motor->set_motor(1, 200);
  delay(2000);
  rolling_basis_ptr->right_motor->set_motor(-1, 200);
  rolling_basis_ptr->left_motor->set_motor(-1, 200);
  delay(2000);
  rolling_basis_ptr->right_motor->set_motor(-1, 0);
  rolling_basis_ptr->left_motor->set_motor(-1, 0);
  delay(2000);
}