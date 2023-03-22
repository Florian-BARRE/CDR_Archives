#include <Arduino.h>
#include <rolling_basis.h>
#include <intercom.h>

// Creation Rolling Basis
#define ENCODER_RESOLUTION 1024
#define CENTER_DISTANCE 27.2
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

float kp = 0.9;
float ki = 0.0003;
float kd = 0.0;
byte max_pwm = 100;

Rolling_Basis *rolling_basis_ptr = new Rolling_Basis(ENCODER_RESOLUTION, CENTER_DISTANCE, WHEEL_DIAMETER, max_pwm, 40);

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
  // put your setup code here, to run once:
  Intercom::init("rolling_basis_1", 115200);

  // Coords topics
  Intercom::subscribe("x");
  Intercom::subscribe("y");
  Intercom::subscribe("theta");
  Intercom::subscribe("new_point");
  Intercom::subscribe("stop");

  // motors init
  rolling_basis_ptr->init_right_motor(R_IN1, R_IN2, R_PWM, R_ENCA, R_ENCB, kp, kd, ki, 1.0);
  rolling_basis_ptr->init_left_motor(L_IN1, L_IN2, L_PWM, L_ENCA, L_ENCB, kp, kd, ki, 1.2);
  rolling_basis_ptr->init_motors();
  attachInterrupt(digitalPinToInterrupt(L_ENCA), left_motor_read_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENCA), right_motor_read_encoder, RISING);
}

float cmd_x = 0.0;
float cmd_y = 0.0;
float cmd_theta = 0.0;
Action current_action = create_action(0.0, 0.0);

void loop()
{
  Intercom::tick();
  int tmp = 0;

  if (Intercom::instantHasReceivedEvent("new_point"))
  {
    current_action = create_action(cmd_x, cmd_y, cmd_theta);
  }
  if (Intercom::instantReceiveFloat("x", &cmd_x)){}
  if (Intercom::instantReceiveFloat("y", &cmd_y)){}
  if (Intercom::instantReceiveFloat("theta", &cmd_theta)){}

  if (Intercom::instantHasReceivedEvent("stop"))
  {
    //current_action = create_action(rolling_basis_ptr->X, rolling_basis_ptr->Y, 0.0);
    delay(5000);
  }


  rolling_basis_ptr->odometrie_handle();
  rolling_basis_ptr->action_handle(&current_action);
  

  /*
  Intercom::publish("print",
    String("start rotation: ") + String(current_action.start_rotation) + String("\n") +
    String("move_forward") + String(current_action.move_forward) + String("\n") +
    String("end_rotation: ") + String(current_action.end_rotation) + String("\n") +
    String("ticks_cursor") + String(current_action.ticks_cursor) + String("\n") +
    String("ticks") + String(current_action.ticks) + String("\n") );*/
  /*
  Serial.println(String("X ") + String(rolling_basis_ptr->X) +String(" Y ") + String(rolling_basis_ptr->Y) + String(" THETA ") + String(rolling_basis_ptr->THETA) + String(" Action -> ") + String(action_index));

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
  }*/

  // Send Odometrie data
  Intercom::publish("x_odo", rolling_basis_ptr->X);
  Intercom::publish("y_odo", rolling_basis_ptr->Y);
  Intercom::publish("theta_odo", rolling_basis_ptr->THETA);

  // Send point to go
  Intercom::publish("x_cmd", cmd_x);
  Intercom::publish("y_cmd", cmd_y);
  Intercom::publish("theta_cmd", cmd_theta);
}