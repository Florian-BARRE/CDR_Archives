#include <Arduino.h>
#include <rolling_basis.h>
#include <util/atomic.h>

// Mouvement params
#define ACTION_ERROR_AUTH 20
#define TRAJECTORY_PRECISION 50
#define NEXT_POSITION_DELAY 10000
#define INACTIVE_DELAY 3000
#define RETURN_START_POSITION_DELAY 999999
#define STOP_MOTORS_DELAY 95000

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

float kp = 1.0;
float ki = 0.5; // 0003
float kd = 0.0;
byte max_pwm = 100;

Rolling_Basis *rolling_basis_ptr = new Rolling_Basis(ENCODER_RESOLUTION, CENTER_DISTANCE, WHEEL_DIAMETER, max_pwm, TRAJECTORY_PRECISION, INACTIVE_DELAY);

#define BALADE_SIZE 8
short action_index = 0;

Action action0 = create_action(0.0, 0.0  , NEXT_POSITION_DELAY, ACTION_ERROR_AUTH);
Action action1 = create_action(30.0, 0.0 , NEXT_POSITION_DELAY, ACTION_ERROR_AUTH);
Action action2 = create_action(30.0, -40.0, NEXT_POSITION_DELAY, ACTION_ERROR_AUTH);
Action action3 = create_action(0.0, -40.0, NEXT_POSITION_DELAY, ACTION_ERROR_AUTH);
Action action4 = create_action(0.0, 0.0, NEXT_POSITION_DELAY, ACTION_ERROR_AUTH);
Action action5 = create_action(0.0, -40.0, NEXT_POSITION_DELAY, ACTION_ERROR_AUTH);
Action action6 = create_action(0.0, -40.0, NEXT_POSITION_DELAY, ACTION_ERROR_AUTH);
Action action7 = create_action(30.0, -40.0, NEXT_POSITION_DELAY, ACTION_ERROR_AUTH);
/*
Action action2 = create_action(50.0, 20.0, NEXT_POSITION_DELAY, ACTION_ERROR_AUTH);
Action action3 = create_action(50.0, 0.0, NEXT_POSITION_DELAY, ACTION_ERROR_AUTH);
*/

Action balade_model[BALADE_SIZE] = {action0, action1, action2, action3, action4, action5, action6, action7};
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
Action return_to_start_position = create_action(0.0, 0.0, NEXT_POSITION_DELAY, ACTION_ERROR_AUTH);

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
  
  // Not end of the game ?
  if (millis() < STOP_MOTORS_DELAY){
    // Authorize to move ?
    if (digitalRead(pin_on_off))
    {
      // Must return to start position ?
      if (millis() > RETURN_START_POSITION_DELAY)
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
      else
        rolling_basis_ptr->keep_current_position(right_ticks, left_ticks);
    }
    else
      rolling_basis_ptr->keep_current_position(right_ticks, left_ticks);
  }
    
  // Print DEBUG part
  byte mvt_state = 0;
  if (balade[action_index].start_rotation) mvt_state = 0;
  if (balade[action_index].move_forward)   mvt_state = 1;
  if (balade[action_index].end_rotation)   mvt_state = 2;

  Serial.println(
      // String("RIGHT :") + String(rolling_basis_ptr->right_motor->ticks) +
      // String(" LEFT :") + String(rolling_basis_ptr->left_motor->ticks) +
      String(" CURSOR ") + String(balade[action_index].ticks_cursor) +
      String(" TICKS ") + String(balade[action_index].ticks) +
      String(" TRIGHT ") + String(balade[action_index].ticks_cursor - abs(rolling_basis_ptr->right_motor->ticks - balade[action_index].right_ref)) +
      String(" TLEFT ") + String(balade[action_index].ticks_cursor - abs(rolling_basis_ptr->left_motor->ticks - balade[action_index].left_ref)) +
      String(" X ") + String(rolling_basis_ptr->X) + String(" Y ") +
      String(rolling_basis_ptr->Y) + String(" THETA ") +
      String(rolling_basis_ptr->THETA) + String(" Action -> ") +
      String(action_index) + String(" cpt -> ") +
      String(balade[action_index].end_movement_cpt) +
      String(" ssState ") + String(mvt_state) +
      String(" State ") + String(digitalRead(pin_on_off))
  );
}