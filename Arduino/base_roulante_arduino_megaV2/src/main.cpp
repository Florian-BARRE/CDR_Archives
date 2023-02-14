#include <Arduino.h>
#include "motors_driver.h"
#include "rolling_basis.h"
#include "action_functions.h"
#include <util/atomic.h>

void move_forward_ticks_calculator(float distance, unsigned long *ticks, short *r_sign, short *l_sign);
void rotation_ticks_calculator(float angle_rotate, unsigned long *ticks, short *r_ticks, short *l_ticks);
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

#define BALADE_SIZE 5
byte action_index = BALADE_SIZE + 1;

Action action0 = create_action(0.0, 0.0, 0.0);
Action action1 = create_action(130.0, 0.0, 0.0);
Action action2 = create_action(130.0, 70.0, 0.0);
Action action3 = create_action(0.0, 70.0, 0.0);
Action action4 = create_action(0.0, 0.0, 0.0);

Action balade_model[BALADE_SIZE] = {action0, action1, action2, action3, action4};
/*
#define BALADE_SIZE 2
byte action_index = BALADE_SIZE + 1;

Action action0 = create_action(0.0, 0.0, 0.0);
Action action1 = create_action(130.0, 0.0, 0.0);

Action balade_model[BALADE_SIZE] = {action0, action1};*/
Action balade[BALADE_SIZE];

/******* Attach Interrupt *******/
void left_motor_read_encoder()
{
  if (digitalRead(L_ENCB) > 0)
    left_motor.ticks++;
  else
    left_motor.ticks--;
}

void right_motor_read_encoder()
{
  if (digitalRead(R_ENCB) > 0)
    right_motor.ticks++;
  else
    right_motor.ticks--;
}

void setup(){
  Serial.begin(9600);
  left_motor.init();
  attachInterrupt(digitalPinToInterrupt(L_ENCA), left_motor_read_encoder, RISING);
  right_motor.init();
  attachInterrupt(digitalPinToInterrupt(R_ENCA), right_motor_read_encoder, RISING);
}

void loop(){

  if (0 <= action_index && action_index < BALADE_SIZE)
  {
    robot_position_calculator();
    mesurement_taker(&balade[action_index]);
    action_supervisor(&balade[action_index]);
    motors_controller(&balade[action_index], max_pwm);
  }
/*
  Serial.print(" ");
  Serial.print(action_index);
  Serial.print(" ");
  Serial.print(balade[action_index].start_rotation);
  Serial.print(" ");
  Serial.print(balade[action_index].move_forward);
  Serial.print(" ");
  Serial.print(balade[action_index].end_rotation);
  Serial.print(" ");
  Serial.print(balade[action_index].end_movement_cpt);
  Serial.print(" ");
  Serial.print(X);
  Serial.print(" ");
  Serial.print(Y);
  Serial.print(" ");
  Serial.print(THETA);*/

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

    unsigned long cmd_ticks;
    //unsigned long ticks_cursor;
    short cmd_right_sign;
    short cmd_left_sign;

    // Rotation mesurment
    if (action->start_rotation || action->end_rotation)
      rotation_ticks_calculator(cmd_theta, &cmd_ticks, &cmd_right_sign, &cmd_left_sign);

    // Go forward mesurement
    else if (action->move_forward)
      move_forward_ticks_calculator(cmd_hypothenuse, &cmd_ticks, &cmd_right_sign, &cmd_left_sign);

    // Update action structure
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      action->right_ref = right_motor.ticks;
      action->left_ref = left_motor.ticks;
    }

    action->mesurement_taker = false;
    action->ticks = cmd_ticks;
    action->ticks_cursor = 0;
    action->right_sign = cmd_right_sign;
    action->left_sign = cmd_left_sign;
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
  long right_ticks;
  long left_ticks;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    right_ticks = right_motor.ticks;
    left_ticks = left_motor.ticks;
  }
  long right_error = abs(action->right_ref + action->right_sign * action->ticks) - abs(right_ticks);
  long left_error = abs(action->left_ref + action->left_sign * action->ticks) - abs(left_ticks);

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
  long right_error;
  long left_error;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    right_error = abs((action->right_ref + action->right_sign * action->ticks) - right_motor.ticks);
    left_error = abs((action->left_ref + action->left_sign * action->ticks) - left_motor.ticks);
  }

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
  int auth_error = 5000;

  long right_ticks;
  long left_ticks;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    right_ticks = right_motor.ticks;
    left_ticks = left_motor.ticks;
  }

  // Verif si beosin de corriger traj
  long right_error = action->ticks_cursor - abs(right_ticks - action->right_ref);
  long left_error = action->ticks_cursor - abs(left_ticks - action->left_ref);

  if (abs(right_error) < auth_error && abs(left_error) < auth_error) {
    if (abs(action->ticks_cursor - action->ticks) < action->error_precision)
      action->ticks_cursor = action->ticks;
    else
      action->ticks_cursor += auth_error;
    
    if (action->ticks_cursor > action->ticks)
      action->ticks_cursor = action->ticks;
  }

  float delta_theta;
  float _;
  trajectory_mesurement_calculator(
      action->target_x,
      action->target_y,
      action->target_theta,
      action->start_rotation,
      action->end_rotation,
      &_,
      &delta_theta);

  float factor = 1.0 - sqrt(map(fabs(delta_theta), 0, PI, 0, 1));
  Serial.println(String("DELTA ") + String(factor));
  if(delta_theta < 0){
    right_motor.handle(deltaT, (action->right_ref + action->right_sign * action->ticks_cursor), max_pwm, 1.0);
    Serial.print(" ");
    left_motor.handle(deltaT, (action->left_ref + action->left_sign * action->ticks_cursor), max_pwm, factor);
  }
  else{
    right_motor.handle(deltaT, (action->right_ref + action->right_sign * action->ticks_cursor), max_pwm, factor);
    Serial.print(" ");
    left_motor.handle(deltaT, (action->left_ref + action->left_sign * action->ticks_cursor), max_pwm, 1.0);
  }

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
    if (x_error == 0 && y_error > 0)
      *cmd_theta = PI / 2;
    else if (x_error == 0 && y_error < 0)
      *cmd_theta = -(PI / 2);

    if (y_error == 0)
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
  //*cmd_theta = (*cmd_theta > PI || *cmd_theta < -PI) ? fmod(*cmd_theta, PI) : *cmd_theta; // Noah a eu la merveilleuse idée d'employé un ?
  *cmd_theta = (*cmd_theta > PI || *cmd_theta < -PI) ? fmod(*cmd_theta, PI) : *cmd_theta;
}

void rotation_ticks_calculator(float angle_rotate, unsigned long *ticks, short *r_sign, short *l_sign)
{
  float distance = (angle_rotate * RADIUS);
  *ticks = (ENCODER_RESOLUTION / WHEEL_PERIMETER) * distance;

  if(*ticks > 0){
    *r_sign = 1;
    *l_sign = -1;
  }
  else {
    *r_sign = -1;
    *l_sign = 1;
  }
}

void move_forward_ticks_calculator(float distance, unsigned long *ticks, short *r_sign, short *l_sign)
{
  *ticks = (ENCODER_RESOLUTION / WHEEL_PERIMETER) * distance;

  if(*ticks > 0){
    *r_sign = 1;
    *l_sign = 1;
  }
  else{
    *r_sign = -1;
    *l_sign = -1;
  }
}
