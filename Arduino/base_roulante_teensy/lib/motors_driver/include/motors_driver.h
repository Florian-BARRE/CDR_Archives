#include <Arduino.h>
#include <Encoder.h>

#define DEBUG_MODE false

class Motor {

private:
    // Pins Motor
    byte _pin_forward;
    byte _pin_backward;
    byte _pin_pwm;  // PWM pin only !

    // Encoder object
    Encoder* _encoder_ptr;

    // PID constantes
    float _kp;
    float _kd;
    float _ki;

    // Variables
    int _error_prev = 0;
    float _error_integral = 0;

    // Debug dunction
    void _debug_print(float dtime, long target_pos, byte max_pwm, int error, double dedt, float integral_error, float cmd);

public:
    // Position in ticks variable
    volatile long ticks = 0;

    // Natural constructor
    Motor(byte pin_forward, byte pin_backward, byte pin_pwm, byte pin_enca, byte pin_encb, float kp, float kd, float ki);
  
    void init();

    void set_motor(int8_t dir, byte pwmVal, byte pwm, byte in1, byte in2);

    void handle(double delta_time, long target_pos, byte max_speed);
};