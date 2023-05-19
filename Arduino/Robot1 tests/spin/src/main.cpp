#include <Arduino.h>
#include <TimerOne.h>
#include <rolling_basis.h>
#include <util/atomic.h>

// Mouvement params
#define ACTION_ERROR_AUTH 20
#define TRAJECTORY_PRECISION 50
#define NEXT_POSITION_DELAY 100
#define INACTIVE_DELAY 2000
#define RETURN_START_POSITION_DELAY 70000
#define STOP_MOTORS_DELAY 98000
#define DISTANCE_NEAR_START_POSITION 30.0

// Creation Rolling Basis
#define ENCODER_RESOLUTION 1024
#define CENTER_DISTANCE 27.07
#define WHEEL_DIAMETER 6.1
 
// Motor Left
#define L_ENCA 12
#define L_ENCB 11
#define L_PWM 5
#define L_IN2 3
#define L_IN1 4

// Motor Right
#define R_ENCA 14
#define R_ENCB 13
#define R_PWM 2
#define R_IN2 1
#define R_IN1 0

float kp = 8.0; // 6
float ki = 0.0; // 0003
float kd = 0.2; // 0.2

byte max_pwm = 150;

Rolling_Basis *rolling_basis_ptr = new Rolling_Basis(ENCODER_RESOLUTION, CENTER_DISTANCE, WHEEL_DIAMETER, max_pwm, TRAJECTORY_PRECISION, INACTIVE_DELAY);

/* Strat part */
#define STRAT_SIZE 4

const Action bleu_strat_model[STRAT_SIZE] = {
    create_action(130.0, 0.0, NEXT_POSITION_DELAY, ACTION_ERROR_AUTH),
    create_action(130.0, 47.0, NEXT_POSITION_DELAY, ACTION_ERROR_AUTH),
    create_action(220.0, 47.0, NEXT_POSITION_DELAY, ACTION_ERROR_AUTH),
    create_action(20.0, 15.0, NEXT_POSITION_DELAY, ACTION_ERROR_AUTH)};
Action bleu_return_position = create_action(20.0, 15.0, NEXT_POSITION_DELAY, ACTION_ERROR_AUTH);

const Action green_strat_model[STRAT_SIZE] = {
    create_action(130.0, 0.0, NEXT_POSITION_DELAY, ACTION_ERROR_AUTH),
    create_action(130.0, -47.0, NEXT_POSITION_DELAY, ACTION_ERROR_AUTH),
    create_action(220.0, -47.0, NEXT_POSITION_DELAY, ACTION_ERROR_AUTH),
    create_action(20.0, -15.0, NEXT_POSITION_DELAY, ACTION_ERROR_AUTH)};
Action green_return_position = create_action(20.0, -15.0, NEXT_POSITION_DELAY, ACTION_ERROR_AUTH);

Action strat[STRAT_SIZE];
Action return_position;
short action_index = 0;

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

// Switch side
byte pin_green_side = 18;

// Globales variables 
long right_ticks = 0;
long left_ticks = 0;

long start_time = -1;

void handle();

void setup()
{
  pinMode(pin_on_off, INPUT);
  pinMode(pin_green_side, INPUT);

  // motors init
  analogWriteFrequency(R_PWM, 60000); 
  analogWriteFrequency(L_PWM, 60000); 

  rolling_basis_ptr->init_right_motor(R_IN1, R_IN2, R_PWM, R_ENCA, R_ENCB, kp, kd, ki, 1.0, 0);
  rolling_basis_ptr->init_left_motor(L_IN1, L_IN2, L_PWM, L_ENCA, L_ENCB, kp, kd, ki, 1.17, 0);
  rolling_basis_ptr->init_motors();
  attachInterrupt(digitalPinToInterrupt(L_ENCA), left_motor_read_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENCA), right_motor_read_encoder, RISING);

  // Init start with the good side
  if (digitalRead(pin_green_side) == 0){
    memcpy(strat, green_strat_model, sizeof(Action) * STRAT_SIZE);
    return_position = green_return_position;
  }
  else{
    memcpy(strat, bleu_strat_model, sizeof(Action) * STRAT_SIZE);
    return_position = bleu_return_position;
  }
    
  // Init motors handle timer
  Timer1.initialize(5000);
  Timer1.attachInterrupt(handle);
}

void loop()
{
  rolling_basis_ptr->odometrie_handle();
}

void handle(){
  rolling_basis_ptr->keep_current_position(287376, -287376);
}