#include <Arduino.h>

// Robots caracteristiques (distance in cm):
#define ENCODER_RESOLUTION 1024
#define CENTER_DISTANCE 29.0
#define WHEEL_DIAMETER 6.25

// Inferred dimensions
#define RADIUS (CENTER_DISTANCE / 2.0)
#define WHEEL_PERIMETER (PI * WHEEL_DIAMETER)
#define WHEEL_UNIT_TICK_CM (WHEEL_PERIMETER / ENCODER_RESOLUTION)

// Pins definition
/*// Motor Left
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

// PID constantes
float kp = 2.0;
float ki = 0.0003;   
float kd = 0.0; 
byte max_pwm = 30;

Motor left_motor(L_IN1, L_IN2, L_PWM, L_ENCA, L_ENCB, kp, kd, ki, 1.2);
Motor right_motor(R_IN1, R_IN2, R_PWM, R_ENCA, R_ENCB, kp, kd, ki, 1.0);