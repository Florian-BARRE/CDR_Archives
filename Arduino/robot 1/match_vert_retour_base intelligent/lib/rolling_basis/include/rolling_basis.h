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
    bool IS_RUNNING = false;
    long last_running_check = 0;
    long last_position_update = 0;
    long running_check_right = 0;
    long running_check_left = 0;
    long inactive_delay = 0;

    long left_ticks = 0;
    long right_ticks = 0;

    unsigned short corrector_error_auth;

    unsigned short encoder_resolution;
    float center_distance;
    float wheel_diameter;


    byte max_pwm;

    Rolling_Basis(unsigned short encoder_resolution, float center_distance, float wheel_diameter, byte max_pwm, unsigned short corrector_error_auth, long inactive_delay);

    void init_position(float x, float y, float theta);
    void init_right_motor(byte enca, byte encb, byte pwm, byte in2, byte in1, float kp, float kd, float ki, float correction_factor, byte threshold_pwm_value);
    void init_left_motor(byte enca, byte encb, byte pwm, byte in2, byte in1, float kp, float kd, float ki, float correction_factor, byte threshold_pwm_value);
    void init_motors();

    void odometrie_handle(); 

    void is_running_update();

    float trajectory_theta_calculator(float target_x, float target_y);
    float delta_theta_calculator(float target_theta);
    float trajectory_distance_calculator(float target_x, float target_y);

    // Angle to ticks
    void theta_to_ticks(float theta_to_rotate, long *ticks, short *r_sign, short *l_sign);
    // Distance to ticks
    void distance_to_ticks(float distance, long *ticks, short *r_sign, short *l_sign);


    // Actions part
    void keep_current_position(long right_ticks, long left_ticks);
    void action_handle(Action *current_action);
    void ticks_calculator(Action *action);
    void motors_controller(Action *action);
    void move_forward_supervisor(Action *action);
    void rotation_supervisor(Action *action);
};