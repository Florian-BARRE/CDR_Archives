#include <motors_driver.h>
#include <Arduino.h>
#include <util/atomic.h>

Motor::Motor(byte pin_forward, byte pin_backward, byte pin_pwm, byte pin_enca, byte pin_encb, float kp, float kd, float ki, float correction_factor = 1.0, byte threshold_pwm_value=0)
{
    _pin_forward = pin_forward;
    _pin_backward = pin_backward;
    _pin_pwm = pin_pwm; // PWM pin only !
    _pin_enca = pin_enca; // AttachInterrupt pin only !
    _pin_encb = pin_encb; // AttachInterrupt pin only !

    _kp = kp;
    _kd = kd;
    _ki = ki;

    _correction_factor = correction_factor;
    _threshold_pwm_value = threshold_pwm_value;
}

void Motor::init(){
    pinMode(_pin_forward, OUTPUT);
    pinMode(_pin_backward, OUTPUT);
    pinMode(_pin_pwm, OUTPUT);

    pinMode(_pin_enca, INPUT);
    pinMode(_pin_encb, INPUT);
}

void Motor::set_motor(int8_t dir, byte pwmVal)
{
    analogWrite(_pin_pwm, pwmVal);
    if (dir == 1)
    {
        digitalWrite(_pin_forward, HIGH);
        digitalWrite(_pin_backward, LOW);
    }
    else if (dir == -1)
    {
        digitalWrite(_pin_forward, LOW);
        digitalWrite(_pin_backward, HIGH);
    }
    else
    {
        digitalWrite(_pin_forward, LOW);
        digitalWrite(_pin_backward, LOW);
    }
}

void Motor::_debug_print(float dtime, long target_pos, byte max_pwm, int error, double dedt, float integral_error, float cmd)
{
    Serial.println("\n###-- DEBUG Motor --###");
    Serial.println(String("#-> Delta time: ") + String(dtime));
    Serial.println(String("#-> Target position in ticks: ") + String(target_pos));
    Serial.println(String("#-> Max speed (pwm max cycle): ") + String(max_pwm));
    Serial.println(String("#-> Position error: ") + String(error));
    Serial.println(String("#-> Derivative error: ") + String(dedt));
    Serial.println(String("#-> Integral error: ") + String(integral_error));
    Serial.println(String("#-> Motor order send: ") + String(cmd));
}

void Motor::handle(double delta_time, long target_pos, byte max_speed)
{
    long fix_ticks = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        fix_ticks = ticks;
    }
    // Calculate error
    int error = fix_ticks - target_pos;

    // Calculate derivative
    double dedt = (error - _error_prev) / delta_time;

    // Calculate integral
    _error_integral = _error_integral + (error * delta_time);

    // Control signal
    float u = _kp * error + _kd * dedt + _ki * _error_integral;

    // Motor power
    float power = fabs(u * _correction_factor);
    if (power > max_speed)
        power = max_speed;

    power += _threshold_pwm_value;
    if(power > 255) power = 255;

    // Motor direction
    int8_t direction = 1;
    if (u < 0)
        direction = -1;

    // Set the correct motor commande
    set_motor(direction, power);

    // Save error
    _error_prev = error;

    /*
    Serial.print(fix_ticks);
    Serial.print(" ");
    Serial.print(target_pos);
    */

    if (DEBUG_MODE)
        _debug_print(delta_time, target_pos, max_speed, error, dedt, _error_integral, u);
}

