#include <Arduino.h>

// Robots caracteristiques (distance in cm):
#define ENCODER_RESOLUTION 1024
#define CENTER_DISTANCE 29.9
#define WHEEL_DIAMETER 6.25

// Inferred dimensions
#define RADIUS (CENTER_DISTANCE / 2)
#define WHEEL_PERIMETER (PI * WHEEL_DIAMETER)
#define WHEEL_UNIT_TICK_CM (WHEEL_PERIMETER / ENCODER_RESOLUTION)

// Pins definition

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
float kp = 0.5;
float ki = 0.0;     // ki = 3.0
float kd = 0.00005; // kd = 0.055;
byte max_pwm = 50;

Motor left_motor(L_IN1, L_IN2, L_PWM, L_ENCA, L_ENCB, kp, kd, ki);
Motor right_motor(R_IN1, R_IN2, R_PWM, R_ENCA, R_ENCB, kp, kd, ki);