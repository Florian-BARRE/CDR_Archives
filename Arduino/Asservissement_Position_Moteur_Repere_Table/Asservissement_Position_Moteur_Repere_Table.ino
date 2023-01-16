#include <util/atomic.h> 
#include "lib.h"

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

// Robots caracteristiques (distance in cm):
#define ENCODER_RESOLUTION 1024
#define CENTER_DISTANCE 32.7
#define WHEEL_DIAMETER 6.3

// Inferred dimensions
#define RADIUS (CENTER_DISTANCE / 2)
#define WHEEL_PERIMETER (PI * WHEEL_DIAMETER)
#define WHEEL_UNIT_TICK_CM (WHEEL_PERIMETER / ENCODER_RESOLUTION)

// Structure Action
struct Action { 
  float target_x;
  float target_y;
  float target_theta;
  bool rotation;
  bool move_forward;
  bool mesurement_taker;
  unsigned int end_movement_cpt;
  unsigned int end_movement_presicion;
  unsigned int error_precision;
  long right_ticks;
  long left_ticks;
};

Action current_action = {0.0, 0.0, 0.0, true, false, true, 0, 1000, 2, 0, 0};
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
 
float kp = 0.8;
float ki = 1.0; //ki = 3.0
float kd = 0.05;// kd = 0.055;
byte max_pwm = 50;

Motor left_motor( L_IN1, L_IN2, L_PWM, L_ENCA, L_ENCB, kp, kd, ki);
Motor right_motor(R_IN1, R_IN2, R_PWM, R_ENCA, R_ENCB, kp, kd, ki);

long prevT = 0;

// Robot position variables
float THETA = 0;
float X     = 0;
float Y     = 0;

long RIGHT_TICKS = 0;
long LEFT_TICKS  = 0;

void setup() {
  Serial.begin(9600);
  
  left_motor.init();
  attachInterrupt(digitalPinToInterrupt(L_ENCA), left_motor_read_encoder, RISING);
  
  right_motor.init();
  attachInterrupt(digitalPinToInterrupt(R_ENCA), right_motor_read_encoder, RISING);
}

/******* Attach Interrupt *******/
void left_motor_read_encoder(){
    if(digitalRead(L_ENCB) > 0) left_motor.volatile_pos++;
    else                        left_motor.volatile_pos--;
}

void right_motor_read_encoder(){
    if(digitalRead(R_ENCB) > 0) right_motor.volatile_pos++;
    else                        right_motor.volatile_pos--;
}


void loop() {
  robot_position_calculator();
  mesurement_taker(&current_action);
  action_supervisor(&current_action);
  motors_controller(&current_action, 100);
}
/******* Actions supervisor *******/
void mesurement_taker(Action* action){
  if(action->mesurement_taker){
    // Trajectoire calculator
    float cmd_hypothenuse;
    float cmd_theta;
    trajectory_mesurement_calculator(
      action->target_x, 
      action->target_y, 
      action->target_theta, 
      &cmd_hypothenuse, 
      &cmd_theta
    );

    long cmd_right_ticks;
    long cmd_left_ticks;
    // Rotation mesurment
    if(action->rotation){
      rotation_ticks_calculator(cmd_theta, &cmd_right_ticks, &cmd_left_ticks);
    }
    // Go forward mesurement
    else if(action->move_forward){
      move_forward_ticks_calculator(cmd_hypothenuse, &cmd_right_ticks, &cmd_left_ticks);
    }

    // Update action structure
    action->mesurement_taker = false;
    action->right_ticks = cmd_right_ticks;
    action->left_ticks  = cmd_left_ticks;
  }
}

void action_supervisor(Action* action){
  if(action->rotation)          rotation_action(action);
  else if(action->move_forward) move_forward_action(action);
}

void rotation_action(Action* action){
  // Mettre dans un ATOMIC BLOCK
  long right_error = abs(action->right_ticks - right_motor.volatile_pos);
  long left_error  = abs(action->left_ticks  - left_motor.volatile_pos );
  
  // Check if the robot is near the target position
  if(action->error_precision <= right_error && action->error_precision <= left_error)
    (action->end_movement_cpt)++;
    
  // Checks if the robot has been in the right position long enough to move to the next action
  if(action->end_movement_cpt >= action->end_movement_presicion){
    action->rotation         = false;
    action->move_forward     = true ;
    action->mesurement_taker = true ;
    action->end_movement_cpt = 0    ;
  }
}

void move_forward_action(Action* action){
  long right_error = abs(action->right_ticks - right_motor.volatile_pos);
  long left_error  = abs(action->left_ticks  - left_motor.volatile_pos );
  
  // Check if the robot is near the target position
  if(action->error_precision <= right_error && action->error_precision <= left_error)
    (action->end_movement_cpt)++;
    
  // Checks if the robot has been in the right position long enough end the action
  if(action->end_movement_cpt >= action->end_movement_presicion){
    action->rotation         = false;
    action->move_forward     = false;
    action->mesurement_taker = false;
    action->end_movement_cpt = 0    ;
  }
}

void motors_controller(Action* action, byte max_pwm){
  float deltaT = delta_time_calculator(prevT);
  
  right_motor.handlee(deltaT, action->right_ticks, max_pwm);
  left_motor.handlee(deltaT , action->left_ticks , max_pwm);
}


/******* Calculator functions *******/
float delta_time_calculator(long &previous_time){
  long current_time = micros();
  float delta_time = ((float) (current_time - previous_time))/( 1.0e6 );
  previous_time = current_time;
  return delta_time;
}

void robot_position_calculator(){
  /* Determine the position of the robot */
  long delta_l_pos = left_motor.volatile_pos  - LEFT_TICKS;  LEFT_TICKS  = LEFT_TICKS  + delta_l_pos;
  long delta_r_pos = right_motor.volatile_pos - RIGHT_TICKS; RIGHT_TICKS = RIGHT_TICKS + delta_r_pos;
  
  float left_move  = delta_l_pos * WHEEL_UNIT_TICK_CM;
  float right_move = delta_r_pos * WHEEL_UNIT_TICK_CM;

  float movement_difference = right_move - left_move;
  float movement_sum        = (right_move + left_move)/2;
  
  THETA = THETA + (movement_difference / CENTER_DISTANCE);
  X = X + (cos(THETA) * movement_sum);
  Y = Y + (sin(THETA) * movement_sum);
}

void trajectory_mesurement_calculator(float target_x, float target_y, float target_theta, float* hypothenuse, float* cmd_theta){
  float x_error   = target_x - X;
  float y_error   = target_y - Y;

  // Calculated info
  *hypothenuse = sqrt((x_error*x_error) + (y_error*y_error));

  if(y_error == 0 || x_error == 0)*cmd_theta = 0;
  
  else *cmd_theta = atan2(y_error, x_error) - THETA;
  /*
  if(*cmd_theta < -PI/2 || *cmd_theta > PI/2){
    *hypothenuse = -(*hypothenuse);
    *cmd_theta = *cmd_theta - PI;
  }*/
}

void rotation_ticks_calculator(float angle_rotate, long* r_ticks, long* l_ticks){
  float distance = (angle_rotate * RADIUS);
  long ticks = (ENCODER_RESOLUTION / WHEEL_PERIMETER) * distance;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      *r_ticks = right_motor.volatile_pos + ticks;
      *l_ticks = left_motor.volatile_pos  - ticks;
  }
}

void move_forward_ticks_calculator(float distance, long* r_ticks, long* l_ticks){
  long ticks = (ENCODER_RESOLUTION / WHEEL_PERIMETER) * distance;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    *r_ticks = right_motor.volatile_pos + ticks;
    *l_ticks = left_motor.volatile_pos  + ticks;
  }
}
