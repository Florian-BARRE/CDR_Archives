/*
#include <Arduino.h>
#include <util/atomic.h>

#include <avr/io.h>
#include <avr/interrupt.h>
*/
#include <Arduino.h>
#include "motors_driver.h"
#include "../../../src/rolling_basis.h"
#include "../../../src/action_functions.h"

void move_forward_ticks_calculator(float distance, long *r_ticks, long *l_ticks);
void rotation_ticks_calculator(float angle_rotate, long *r_ticks, long *l_ticks);
void trajectory_mesurement_calculator(float target_x, float target_y, float target_theta, bool start_rotation, bool end_rotation, float *hypothenuse, float *cmd_theta);
void robot_position_calculator();

float delta_time_calculator(long &previous_time);

void motors_controller(Action *action, byte max_pwm);
void move_forward_action(Action *action);
void rotation_action(Action *action);
void action_supervisor(Action *action);
void mesurement_taker(Action *action);

long prevT = 0;

// Robot position variables
float THETA = 0;
float X = 0;
float Y = 0;
long RIGHT_TICKS = 0;
long LEFT_TICKS = 0;

// Balade creation
#define BALADE_SIZE 3
byte action_index = BALADE_SIZE + 1;

Action action0 = create_action(0.0, 0.0, 0.0);
Action action1 = create_action(0.0, 0.0, PI / 2);
Action action2 = create_action(0.0, 0.0, PI);

Action balade_model[BALADE_SIZE] = {action0, action1, action2};
Action balade[BALADE_SIZE];

void setup()
{
  Serial.begin(9600);
  left_motor.init();
  right_motor.init();
}

void loop()
{
  if (0 <= action_index && action_index < BALADE_SIZE) {
    robot_position_calculator();
    mesurement_taker(&balade[action_index]);
    action_supervisor(&balade[action_index]);
    motors_controller(&balade[action_index], max_pwm);
  }

  // DisplayAction(&balade[action_index]);

  Serial.println(String("INDEX ") + String(action_index));

  // Next coords if the current is done
  if ( balade[action_index].start_rotation == false && balade[action_index].move_forward == false && balade[action_index].end_rotation == false)
    action_index++;
  
  // No out of range
  if (action_index >= BALADE_SIZE)
  {
    action_index = 0;
    memcpy(balade, balade_model, sizeof(Action) * BALADE_SIZE);
  }

  Serial.println();
}
/****-- Supervisor Part --****/
void mesurement_taker(Action *action)
{
  if (action->mesurement_taker)
  {
    // Trajectoire calculator
    float cmd_hypothenuse;
    float cmd_theta;
    trajectory_mesurement_calculator(
        action->target_x,
        action->target_y,
        action->target_theta,
        action->start_rotation,
        action->end_rotation,
        &cmd_hypothenuse,
        &cmd_theta);

    long cmd_right_ticks;
    long cmd_left_ticks;
    // Rotation mesurment
    if (action->start_rotation || action->end_rotation)
      rotation_ticks_calculator(cmd_theta, &cmd_right_ticks, &cmd_left_ticks);

    // Go forward mesurement
    else if (action->move_forward)
      move_forward_ticks_calculator(cmd_hypothenuse, &cmd_right_ticks, &cmd_left_ticks);

    // Update action structure
    action->mesurement_taker = false;
    action->right_ticks = cmd_right_ticks;
    action->left_ticks = cmd_left_ticks;
  }
}

void action_supervisor(Action *action)
{
  if (action->start_rotation)
    rotation_action(action);
  else if (action->move_forward)
    move_forward_action(action);
  else if (action->end_rotation)
    rotation_action(action);
}

void rotation_action(Action *action)
{
  // Mettre dans un ATOMIC BLOCK
  long right_error = abs(action->right_ticks - right_motor.ticks);
  long left_error = abs(action->left_ticks - left_motor.ticks);

  // Check if the robot is near the target position
  if (action->error_precision >= right_error && action->error_precision >= left_error)
    (action->end_movement_cpt)++;

  // Checks if the robot has been in the right position long enough to move to the next action
  if (action->end_movement_cpt >= action->end_movement_presicion)
  {
    if (action->start_rotation)
    {
      action->start_rotation = false;
      action->move_forward = true;
      action->mesurement_taker = true;
    }
    else if (action->end_rotation)
    {
      action->end_rotation = false;
      action->move_forward = false;
      action->mesurement_taker = false;
    }
    action->end_movement_cpt = 0;
  }
}

void move_forward_action(Action *action)
{
  long right_error = abs(action->right_ticks - right_motor.ticks);
  long left_error = abs(action->left_ticks - left_motor.ticks);

  // Check if the robot is near the target position
  if (action->error_precision >= right_error && action->error_precision >= left_error)
    (action->end_movement_cpt)++;

  // Checks if the robot has been in the right position long enough end the action
  if (action->end_movement_cpt >= action->end_movement_presicion)
  {
    action->start_rotation = false;
    action->move_forward = false;
    action->end_rotation = true;
    action->mesurement_taker = true;
    action->end_movement_cpt = 0;
  }
}

void motors_controller(Action *action, byte max_pwm)
{
  double deltaT = delta_time_calculator(prevT);

  right_motor.handle(deltaT, action->right_ticks, max_pwm);
  left_motor.handle(deltaT, action->left_ticks, max_pwm);
}

/******* Calculator functions *******/
float delta_time_calculator(long &previous_time)
{
  long current_time = micros();
  float delta_time = ((float)(current_time - previous_time)) / (1.0e6);
  previous_time = current_time;
  return delta_time;
}

void robot_position_calculator()
{
  /* Determine the position of the robot */
  long delta_l_pos = left_motor.ticks - LEFT_TICKS;
  LEFT_TICKS = LEFT_TICKS + delta_l_pos;
  long delta_r_pos = right_motor.ticks - RIGHT_TICKS;
  RIGHT_TICKS = RIGHT_TICKS + delta_r_pos;

  float left_move = delta_l_pos * WHEEL_UNIT_TICK_CM;
  float right_move = delta_r_pos * WHEEL_UNIT_TICK_CM;

  float movement_difference = right_move - left_move;
  float movement_sum = (right_move + left_move) / 2;

  THETA = THETA + (movement_difference / CENTER_DISTANCE);
  X = X + (cos(THETA) * movement_sum);
  Y = Y + (sin(THETA) * movement_sum);
}

void trajectory_mesurement_calculator(float target_x, float target_y, float target_theta, bool start_rotation, bool end_rotation, float *hypothenuse, float *cmd_theta)
{
  float x_error = target_x - X;
  float y_error = target_y - Y;

  // Calculated info
  *hypothenuse = sqrt((x_error * x_error) + (y_error * y_error));

  // Rotation
  if (start_rotation)
  {
    if (y_error == 0 || x_error == 0)
      *cmd_theta = 0;

    else
      *cmd_theta = atan2(y_error, x_error) - THETA;
  }
  else if (end_rotation)
  {
    *cmd_theta = target_theta - THETA;
  }

  // Took the best rotation angle
  // if(*cmd_theta>PI) *cmd_theta =   -fmod(*cmd_theta, PI);
  *cmd_theta = (*cmd_theta > PI || *cmd_theta < -PI) ? -fmod(*cmd_theta, PI) : *cmd_theta; // Noah a eu la merveilleuse idée d'employé un ?
}

void rotation_ticks_calculator(float angle_rotate, long *r_ticks, long *l_ticks)
{
  float distance = (angle_rotate * RADIUS);
  long ticks = (ENCODER_RESOLUTION / WHEEL_PERIMETER) * distance;

  *r_ticks = right_motor.ticks + ticks;
  *l_ticks = left_motor.ticks - ticks;
}

void move_forward_ticks_calculator(float distance, long *r_ticks, long *l_ticks)
{
  long ticks = (ENCODER_RESOLUTION / WHEEL_PERIMETER) * distance;

  *r_ticks = right_motor.ticks + ticks;
  *l_ticks = left_motor.ticks + ticks;
}
