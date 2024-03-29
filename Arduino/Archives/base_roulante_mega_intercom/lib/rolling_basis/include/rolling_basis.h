#include <Arduino.h>
#include <motors_driver.h>
#include <actions.h>


class Rolling_Basis {

private:
    inline float radius(){return this->center_distance / 2.0; };
    inline float wheel_perimeter() { return this->wheel_diameter * PI; };
    inline float wheel_unit_tick_cm() { return this->wheel_perimeter() / this->encoder_resolution; };

    inline float delta_time_calculator(float &previous_time);

    float prevT;

public :
    Motor *right_motor;
    Motor *left_motor;

    float X = 0.0;
    float Y = 0.0;
    float THETA = 0.0;

    long left_ticks = 0;
    long right_ticks = 0;

    unsigned short corrector_error_auth;

    unsigned short encoder_resolution;
    float center_distance;
    float wheel_diameter;


    byte max_pwm;

    Rolling_Basis(unsigned short encoder_resolution, float center_distance, float wheel_diameter, byte max_pwm, unsigned short corrector_error_auth);

    void init_position(float x, float y, float theta);
    void init_right_motor(byte enca, byte encb, byte pwm, byte in2, byte in1, float kp, float kd, float ki, float correction_factor = 1.0);
    void init_left_motor(byte enca, byte encb, byte pwm, byte in2, byte in1, float kp, float kd, float ki, float correction_factor = 1.0);
    void init_motors();

    void odometrie_handle(); // inline ?

    float trajectory_theta_calculator(float target_x, float target_y);
    float delta_theta_calculator(float target_theta);
    float trajectory_distance_calculator(float target_x, float target_y);

    // Angle to ticks
    void theta_to_ticks(float theta_to_rotate, unsigned long *ticks, short *r_sign, short *l_sign);
    // Distance to ticks
    void distance_to_ticks(float distance, unsigned long *ticks, short *r_sign, short *l_sign);


    // Actions part
    void action_handle(Action *current_action);
    void ticks_calculator(Action *action);
    void motors_controller(Action *action);
    void move_forward_supervisor(Action *action);
    void rotation_supervisor(Action *action);
};