#include <Arduino.h>
#include <rolling_basis.h>
#include <util/atomic.h>

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

float kp = 0.9;
float ki = 0.0003; // 0003
float kd = 0.0;
byte max_pwm = 100;

Rolling_Basis *rolling_basis_ptr = new Rolling_Basis(ENCODER_RESOLUTION, CENTER_DISTANCE, WHEEL_DIAMETER, max_pwm, 50);

#define BALADE_SIZE 5
byte action_index = 0;

Action action0 = create_action(0.0, 0.0);
Action action1 = create_action(200.0, 0.0);
Action action2 = create_action(200.0, 100.0);
Action action3 = create_action(200.0, 0.0);
Action action4 = create_action(0.0, 0.0);

Action balade_model[BALADE_SIZE] = {action0, action1, action2, action3, action4};
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

// Return to start position delay
int return_start_position_delay = 70000;
Action return_to_start_position = create_action(0.0, 0.0);

void setup()
{
  Serial.begin(9600);
  pinMode(pin_on_off, INPUT);

  // motors init
  analogWriteFrequency(R_PWM, 60000); 
  analogWriteFrequency(L_PWM, 60000); 

  rolling_basis_ptr->init_right_motor(R_IN1, R_IN2, R_PWM, R_ENCA, R_ENCB, kp, kd, ki, 1.0, 0);
  rolling_basis_ptr->init_left_motor(L_IN1, L_IN2, L_PWM, L_ENCA, L_ENCB, kp, kd, ki, 1.0, 0);
  rolling_basis_ptr->init_motors();
  attachInterrupt(digitalPinToInterrupt(L_ENCA), left_motor_read_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENCA), right_motor_read_encoder, RISING);

  // Init balade
  memcpy(balade, balade_model, sizeof(Action) * BALADE_SIZE);
}

void loop()
{
  rolling_basis_ptr->odometrie_handle();
  // Authorize to move ?
  if (digitalRead(pin_on_off))
  {
    // Must return to start position ?
    if (millis() < return_start_position_delay && false)
      rolling_basis_ptr->action_handle(&return_to_start_position);
    // Do classic trajectory
    else if (action_index < BALADE_SIZE)
    {
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
      {
        right_ticks = rolling_basis_ptr->right_motor->ticks;
        left_ticks = rolling_basis_ptr->left_motor->ticks;
      }

      // If there is an action to complete
      if (0 <= action_index && action_index < BALADE_SIZE)
        rolling_basis_ptr->action_handle(&balade[action_index]);

      // Next coords if the current is done
      if (balade[action_index].start_rotation == false && balade[action_index].move_forward == false && balade[action_index].end_rotation == false)
        action_index++;
    }
  }
  else
    rolling_basis_ptr->keep_current_position(right_ticks, left_ticks);
}