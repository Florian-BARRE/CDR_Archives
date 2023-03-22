#include <Arduino.h>
#include <rolling_basis.h>

// Creation Rolling Basis
#define ENCODER_RESOLUTION 1024
#define CENTER_DISTANCE 29.0
#define WHEEL_DIAMETER 6.25

// Motor Left
#define L_ENCA 18
#define L_ENCB 19
#define L_PWM 5
#define L_IN2 6
#define L_IN1 7

// Motor Right
#define R_ENCA 20
#define R_ENCB 21
#define R_PWM 10
#define R_IN2 11
#define R_IN1 12

float kp = 2.0;
float ki = 0.0003;
float kd = 0.0;
byte max_pwm = 30;

Rolling_Basis *rolling_basis_ptr = new Rolling_Basis(ENCODER_RESOLUTION, CENTER_DISTANCE, WHEEL_DIAMETER, max_pwm, 50);

// Balade creation
#define BALADE_SIZE 2
byte action_index = BALADE_SIZE + 1;

Action action0 = create_action(0.0, 0.0);
Action action1 = create_action(5.0, 0.0);
Action action2 = create_action(50.0, 0.0);
Action action3 = create_action(50.0, 50.0);
Action action4 = create_action(100.0, 50.0);
Action action5 = create_action(100.0, 0.0);
Action balade_model[BALADE_SIZE] = {action0, action1};
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

void setup()
{
  Serial.begin(9600);
  // motors init
  rolling_basis_ptr->init_right_motor(R_IN1, R_IN2, R_PWM, R_ENCA, R_ENCB, kp, kd, ki, 1.0);
  rolling_basis_ptr->init_left_motor(L_IN1, L_IN2, L_PWM, L_ENCA, L_ENCB, kp, kd, ki, 1.2);
  rolling_basis_ptr->init_motors();
  attachInterrupt(digitalPinToInterrupt(L_ENCA), left_motor_read_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENCA), right_motor_read_encoder, RISING);
}

void loop()
{
  
  rolling_basis_ptr->odometrie_handle();

  if (0 <= action_index && action_index < BALADE_SIZE)
    rolling_basis_ptr->action_handle(&balade[action_index]);

  // Next coords if the current is done
  if (balade[action_index].start_rotation == false && balade[action_index].move_forward == false && balade[action_index].end_rotation == false)
    action_index++;

  // No out of range
  if (action_index >= BALADE_SIZE)
  {
    action_index = 0;
    memcpy(balade, balade_model, sizeof(Action) * BALADE_SIZE);
  }
  Serial.println();
}