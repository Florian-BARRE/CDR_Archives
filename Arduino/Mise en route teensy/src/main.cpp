#include <Arduino.h>
#include <util/atomic.h>

#include <avr/io.h>
#include <avr/interrupt.h>

#include <Encoder.h> // Librairie déjà inclue avec l'installation de teensyduino

class Motor
{

private:
  // Pins Motor
  byte _pin_forward;
  byte _pin_backward;
  byte _pin_pwm;  // PWM pin only !
  byte _pin_enca; // AttachInterrupt pin only !
  byte _pin_encb; // AttachInterrupt pin only !

  // PID constantes
  float _kp;
  float _kd;
  float _ki;

  // Variables
  int _error_prev = 0;
  float _error_integral = 0;

public:
  // Attach interrupt variable
  volatile long volatile_pos = 0;

  // Natural constructor
  Motor(byte pin_forward, byte pin_backward, byte pin_pwm, byte pin_enca, byte pin_encb, float kp, float kd, float ki)
  {
    _pin_forward = pin_forward;
    _pin_backward = pin_backward;
    _pin_pwm = pin_pwm;   // PWM pin only !
    _pin_enca = pin_enca; // AttachInterrupt pin only !
    _pin_encb = pin_encb; // AttachInterrupt pin only !

    _kp = kp;
    _kd = kd;
    _ki = ki;
  }

  void init()
  {
    pinMode(_pin_forward, OUTPUT);
    pinMode(_pin_backward, OUTPUT);
    pinMode(_pin_pwm, OUTPUT);
  }

  void setMotor(int8_t dir, byte pwmVal, byte pwm, byte in1, byte in2)
  {
    analogWrite(pwm, pwmVal);
    if (dir == -1)
    {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    }
    else if (dir == 1)
    {
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
    }
    else
    {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    }
  }

  void handlee(double delta_time, long target_pos, byte max_speed)
  {
    /*
    long pos = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
      pos = volatile_pos;
    }*/
    // Calculate error
    int error = volatile_pos - target_pos;

    // Calculate derivative
    float dedt = (error - _error_prev) / delta_time;

    // Calculate integral
    _error_integral = _error_integral + (error * delta_time);

    // Control signal
    float u = _kp * error + _kd * dedt + _ki * _error_integral;

    // Motor power
    float power = fabs(u);
    if (power > max_speed)
      power = max_speed;

    // Motor direction
    int8_t direction = 1;
    if (u < 0)
      direction = -1;

    /*
      Serial.print(" ");
      Serial.print(target_pos);
      Serial.print(" ");
      Serial.print(volatile_pos);
      Serial.print(" ");
    */

    // Set the correct motor commande
    setMotor(direction, power, _pin_pwm, _pin_forward, _pin_backward);

    // Save error
    _error_prev = error;
  }
};

// Motor Left
#define L_ENCA 8 // blanc  -> jaune 8
#define L_ENCB 7 // violet -> orange 7
#define L_PWM 9  // violet -> vert 9
#define L_IN2 4  // bleu -> blanc 4
#define L_IN1 5  // vert -> gris 5

// Motor Right
#define R_ENCA 6  // marron -> rouge 6
#define R_ENCB 10 // orange -> bleu  10
#define R_PWM 1   // bleu -> orange 1
#define R_IN2 3   // violet -> noir 3
#define R_IN1 2   // blanc -> marron 2

// Robots caracteristiques (distance in cm):
#define ENCODER_RESOLUTION 1024
#define CENTER_DISTANCE 29.9
#define WHEEL_DIAMETER 6.25

// Inferred dimensions
#define RADIUS (CENTER_DISTANCE / 2)
#define WHEEL_PERIMETER (PI * WHEEL_DIAMETER)
#define WHEEL_UNIT_TICK_CM (WHEEL_PERIMETER / ENCODER_RESOLUTION)

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
  long right_ticks;
  long left_ticks;
};

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
void DisplayAction(Action *act);

Encoder R_ENC(R_ENCA, R_ENCB);
Encoder L_ENC(L_ENCA, L_ENCB);

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

float kp = 0.5;
float ki = 0.0; // ki = 3.0
float kd = 0.00005; // kd = 0.055;
byte max_pwm = 50;

Motor left_motor(L_IN1, L_IN2, L_PWM, L_ENCA, L_ENCB, kp, kd, ki);
Motor right_motor(R_IN1, R_IN2, R_PWM, R_ENCA, R_ENCB, kp, kd, ki);

long prevT = 0;

// Robot position variables
float THETA = 0;
float X = 0;
float Y = 0;
long RIGHT_TICKS = 0;
long LEFT_TICKS = 0;



// Balade
Action createAction(float obj_x, float obj_y, float obj_theta)
{
  unsigned int precision_cpt = 10;
  unsigned int error_auth = 5;
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
  new_action.right_ticks = 0;
  new_action.left_ticks = 0;

  //Action new_action = {obj_x, obj_y, obj_theta, true, false, false, true, 0, precision_cpt, error_auth, 0, 0};
  return new_action;
}

#define BALADE_SIZE 3
byte action_index = BALADE_SIZE + 1;

Action action0 = createAction(0.0, 0.0, 0.0);
Action action1 = createAction(0.0, 0.0, PI/2);
Action action2 = createAction(0.0, 0.0, PI);

Action balade_model[BALADE_SIZE] = {action0, action1, action2};
Action balade[BALADE_SIZE];

void setup()
{
  Serial.begin(9600);
  left_motor.init();
  right_motor.init();
  Serial.print(" SETUP" );
  DisplayAction(&action0);
}

void loop()
{
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    left_motor.volatile_pos = L_ENC.read();
    right_motor.volatile_pos = R_ENC.read();
  }
  if (0 <= action_index && action_index < BALADE_SIZE)
  {
    robot_position_calculator();
    Serial.print(" A ");
    DisplayAction(&balade[action_index]);
    mesurement_taker(&balade[action_index]);
    Serial.print(" B ");
    DisplayAction(&balade[action_index]);
    action_supervisor(&balade[action_index]);
    Serial.print(" C ");
    DisplayAction(&balade[action_index]);
    motors_controller(&balade[action_index], max_pwm);
    Serial.print(" D "); DisplayAction(&balade[action_index]);
    //delay(1000);
  }

 // DisplayAction(&balade[action_index]);

  Serial.println(String("INDEX ") + String(action_index));

  // Next coords if the current is done
  if (
      balade[action_index].start_rotation == false &&
      balade[action_index].move_forward == false &&
      balade[action_index].end_rotation == false)
    { action_index++; }
   

  // No out of range
  if (action_index >= BALADE_SIZE)
  {
    action_index = 0;
    memcpy(balade, balade_model, sizeof(Action) * BALADE_SIZE);
  }

  Serial.println();
}

/******* Actions supervisor *******/
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
        &cmd_theta
      );

    long cmd_right_ticks;
    long cmd_left_ticks;
    // Rotation mesurment
    if (action->start_rotation || action->end_rotation)
    {
      rotation_ticks_calculator(cmd_theta, &cmd_right_ticks, &cmd_left_ticks);
    }
    // Go forward mesurement
    else if (action->move_forward)
    {
      move_forward_ticks_calculator(cmd_hypothenuse, &cmd_right_ticks, &cmd_left_ticks);
    }

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
  long right_error = abs(action->right_ticks - right_motor.volatile_pos);
  long left_error = abs(action->left_ticks - left_motor.volatile_pos);

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
  long right_error = abs(action->right_ticks - right_motor.volatile_pos);
  long left_error = abs(action->left_ticks - left_motor.volatile_pos);

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

  right_motor.handlee(deltaT, action->right_ticks, max_pwm);
  left_motor.handlee(deltaT, action->left_ticks, max_pwm);
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
  long delta_l_pos = left_motor.volatile_pos - LEFT_TICKS;
  LEFT_TICKS = LEFT_TICKS + delta_l_pos;
  long delta_r_pos = right_motor.volatile_pos - RIGHT_TICKS;
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
  if(start_rotation){
    if (y_error == 0 || x_error == 0)
      *cmd_theta = 0;

    else
      *cmd_theta = atan2(y_error, x_error) - THETA;
  }
  else if (end_rotation){
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

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    *r_ticks = right_motor.volatile_pos + ticks;
    *l_ticks = left_motor.volatile_pos - ticks;
  }
}

void move_forward_ticks_calculator(float distance, long *r_ticks, long *l_ticks)
{
  long ticks = (ENCODER_RESOLUTION / WHEEL_PERIMETER) * distance;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    *r_ticks = right_motor.volatile_pos + ticks;
    *l_ticks = left_motor.volatile_pos + ticks;
  }
}

void DisplayAction(Action *act)
{
  Serial.println();
  Serial.print("target_x");
  Serial.println(act->target_x);
  Serial.print("target_y");
  Serial.println(act->target_y);
  Serial.print("target_theta");
  Serial.println(act->target_theta);
  Serial.print("start rotation");
  Serial.println(act->start_rotation);
  Serial.print("move_forward");
  Serial.println(act->move_forward);
  Serial.print("end rotation");
  Serial.println(act->end_rotation);
  Serial.print("mesurement_taker");
  Serial.println(act->mesurement_taker);
  Serial.print("end_movement_cpt");
  Serial.println(act->end_movement_cpt);
  Serial.print("end_movement_presicion");
  Serial.println(act->end_movement_presicion);
  Serial.print("error_precision");
  Serial.println(act->error_precision);
  Serial.print("right_ticks");
  Serial.println(act->right_ticks);
  Serial.print("left_ticks");
  Serial.println(act->left_ticks);
  Serial.println();
}