#include <util/atomic.h> 
#include "lib.h"

/*
// Motor Left
#define L_ENCA 18
#define L_ENCB 19
#define L_PWM 10
#define L_IN2 11
#define L_IN1 12

// Motor Right
#define R_ENCA 20
#define R_ENCB 21
#define R_PWM 5
#define R_IN2 6
#define R_IN1 7
*/

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

// Robots caracteristiques (distance in cm):
#define ENCODER_RESOLUTION 1024
#define CENTER_DISTANCE 29.1
#define WHEEL_DIAMETER 6.25

// Inferred dimensions
#define RADIUS (CENTER_DISTANCE / 2)
#define WHEEL_PERIMETER (PI * WHEEL_DIAMETER)
#define WHEEL_UNIT_TICK_CM (WHEEL_PERIMETER / ENCODER_RESOLUTION)

#define TICKS_SIZE 5000
// Structure Action
struct Action
{
    float target_x;
    float target_y;
    float target_theta;
    bool start_rotation;
    bool move_forward;
    bool end_rotation;
    bool mesurement_taker;
    unsigned int end_movement_cpt;
    unsigned int end_movement_presicion;
    unsigned int error_precision;
    int ticks_index;
    long ticks_mvt[TICKS_SIZE];
    short right_sign;
    short left_sign;
};

/* Set the PID
 * kp -> proportional
 * /-> Allows you to set the reaction speed of the system 
 * ki -> integral
 * /-> Allows to reduce the static error (in steady state)
 * kd -> differential 
 * /-> Allows to improve the stability of the system
 *  
 * Adjustment order :
 * I) Kp
 * II) Ki
 * III)Kd
 */
 
float kp = 1.5;
float ki = 0.001; //ki = 3.0
float kd = 0.05;// kd = 0.055;
byte max_pwm = 50;

Motor left_motor( L_IN1, L_IN2, L_PWM, L_ENCA, L_ENCB, kp, kd, ki, 0);
Motor right_motor(R_IN1, R_IN2, R_PWM, R_ENCA, R_ENCB, kp, kd, ki, 0);

long prevT = 0;

// Robot position variables
float THETA = 0;
float X = 0;
float Y = 0;
long RIGHT_TICKS = 0;
long LEFT_TICKS = 0;

// Action creation
Action create_action(float obj_x, float obj_y, float obj_theta)
{
    unsigned int precision_cpt = 100;
    unsigned int error_auth = 10;
    Action new_action;
    new_action.target_x = obj_x;
    new_action.target_y = obj_y;
    new_action.target_theta = obj_theta;
    new_action.start_rotation = true;
    new_action.move_forward = false;
    new_action.end_rotation = false;
    new_action.mesurement_taker = true;
    new_action.end_movement_cpt = 0;
    new_action.end_movement_presicion = precision_cpt;
    new_action.error_precision = error_auth;
    //new_action.right_ticks = 0;
    //new_action.left_ticks = 0;
    new_action.ticks_index = 0;
    new_action.right_sign = 1;
    new_action.left_sign = 1;

    // Action new_action = {obj_x, obj_y, obj_theta, true, false, false, true, 0, precision_cpt, error_auth, 0, 0};
    return new_action;
}

// Balade creation
#define BALADE_SIZE 5
byte action_index = BALADE_SIZE + 1;

Action action0 = create_action(0.0, 0.0, 0.0);
Action action1 = create_action(50.0, 0.0, 0.0);
Action action2 = create_action(0.0, 0.0, 0.0);
Action action3 = create_action(50.0, 0.0, 0.0);
Action action4 = create_action(0.0, 0.0, 0.0);


Action balade_model[BALADE_SIZE] = {action0, action1, action2, action3, action4};
Action balade[BALADE_SIZE];

void setup() {
  Serial.begin(9600);
  
  left_motor.init();
  attachInterrupt(digitalPinToInterrupt(L_ENCA), left_motor_read_encoder, RISING);
  
  right_motor.init();
  attachInterrupt(digitalPinToInterrupt(R_ENCA), right_motor_read_encoder, RISING);
}

/******* Attach Interrupt *******/
void left_motor_read_encoder(){
    if(digitalRead(L_ENCB) > 0) left_motor.ticks++;
    else                        left_motor.ticks--;
}

void right_motor_read_encoder(){
    if(digitalRead(R_ENCB) > 0) right_motor.ticks++;
    else                        right_motor.ticks--;
}
void loop()
{

  if (0 <= action_index && action_index < BALADE_SIZE)
   {
     robot_position_calculator();
     mesurement_taker(&balade[action_index]);
     action_supervisor(&balade[action_index]);
     motors_controller(&balade[action_index], max_pwm);
   }

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
   Serial.print(THETA);

   // Next coords if the current is done
   if ( balade[action_index].start_rotation == false && balade[action_index].move_forward == false && balade[action_index].end_rotation == false)
     action_index++;

   // No out of range
   if (action_index >= BALADE_SIZE)
   {
     action_index = 0;
     memcpy(balade, balade_model, sizeof(Action) * BALADE_SIZE);
   }
  
  /*
  Serial.print(" ");
  Serial.print(action_index);
  Serial.print(" ");
  Serial.print(action0.start_rotation);
  Serial.print(" ");
  Serial.print(action0.move_forward);
  Serial.print(" ");
  Serial.print(action0.end_rotation);
  Serial.print(" ");
  Serial.print(action0.end_movement_cpt);
  Serial.print(" ");
  Serial.print(X);
  Serial.print(" ");
  Serial.print(Y);
  Serial.print(" ");
  Serial.print(THETA);
  Serial.println();
  */
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
        
    long ticks_mvt[TICKS_SIZE];
    short right_sign;
    short left_sign;
    // Rotation mesurment
    if (action->start_rotation || action->end_rotation)
      rotation_ticks_calculator(cmd_theta, ticks_mvt, &right_sign, &left_sign);

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
  Serial.print(" ");
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
    if (x_error == 0 && y_error > 0)
      *cmd_theta = PI/2;
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
  *cmd_theta = (*cmd_theta > PI || *cmd_theta < -PI) ? -fmod(*cmd_theta, PI) : *cmd_theta; // Noah a eu la merveilleuse idée d'employé un ?
}

void rotation_ticks_calculator(float angle_rotate, long ticks[], short *right_sign, short *left_sign)
{
  float distance = (angle_rotate * RADIUS);
  long nb_ticks = (ENCODER_RESOLUTION / WHEEL_PERIMETER) * distance;

  for(unsigned k=0; k< TICKS_SIZE; k++){
    if( k <= nb_ticks)
      ticks[k] = k;
    else
      ticks[k] = nb_ticks;
  }
  *r_ticks = right_motor.ticks + ticks;
  *l_ticks = left_motor.ticks - ticks;

  if(nb_ticks < 0){
    *right_sign = 1;
    *left_sign  = -1;
  }
  else{
    *right_sign = -1;
    *left_sign  = 1;
  }
  
}

void move_forward_ticks_calculator(float distance, long *r_ticks, long *l_ticks)
{
  long ticks = (ENCODER_RESOLUTION / WHEEL_PERIMETER) * distance;

  *r_ticks = right_motor.ticks + ticks;
  *l_ticks = left_motor.ticks + ticks;
}
