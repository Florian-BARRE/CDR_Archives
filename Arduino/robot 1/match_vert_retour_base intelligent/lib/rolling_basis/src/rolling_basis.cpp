#include <rolling_basis.h>
#include <Arduino.h>
#include <util/atomic.h>

inline float Rolling_Basis::delta_time_calculator(float &previous_time){
    long current_time = micros();
    float delta_time = ((float)(current_time - previous_time)) / (1.0e6);
    previous_time = current_time;
    return delta_time;
}

void Rolling_Basis::init_position(float x, float y, float theta)
{
    this->X=x;
    this->Y=y;
    this->THETA=theta;
}

void Rolling_Basis::init_motors(){
    this->right_motor->init();
    this->left_motor->init();
}

Rolling_Basis::Rolling_Basis(unsigned short encoder_resolution, float center_distance, float wheel_diameter, byte max_pwm, unsigned short corrector_error_auth, long inactive_delay)
{
    this->encoder_resolution = encoder_resolution;
    this->center_distance = center_distance;
    this->wheel_diameter = wheel_diameter;
    this->max_pwm = max_pwm;
    this->corrector_error_auth = corrector_error_auth;
    this->inactive_delay = inactive_delay;
}

void Rolling_Basis::init_right_motor(byte enca, byte encb, byte pwm, byte in2, byte in1, float kp, float kd, float ki, float correction_factor = 1.0, byte threshold_pwm_value=0)
{
    this->right_motor = new Motor(enca, encb, pwm, in2, in1, kp, kd, ki, correction_factor, threshold_pwm_value);
}

void Rolling_Basis::init_left_motor(byte enca, byte encb, byte pwm, byte in2, byte in1, float kp, float kd, float ki, float correction_factor = 1.0, byte threshold_pwm_value = 0)
{
    this->left_motor = new Motor(enca, encb, pwm, in2, in1, kp, kd, ki, correction_factor, threshold_pwm_value);
}

void Rolling_Basis::odometrie_handle(){
    /* Determine the position of the robot */
    long delta_left  = this->left_motor->ticks - this->left_ticks;
    this->left_ticks = this->left_ticks + delta_left;

    long delta_right  = this->right_motor->ticks - this->right_ticks;
    this->right_ticks = this->right_ticks + delta_right;
    
    float left_move  = delta_left * this->wheel_unit_tick_cm();
    float right_move = delta_right * this->wheel_unit_tick_cm();

    float movement_difference = right_move - left_move;
    float movement_sum = (right_move + left_move) / 2;

    THETA = THETA + (movement_difference / this->center_distance);
    this->X = this->X + (cos(this->THETA) * movement_sum);
    this->Y = this->Y + (sin(this->THETA) * movement_sum);
}

void Rolling_Basis::is_running_update(){
    if ((millis() - this->last_running_check) > 10){
        this->last_running_check = millis();
        long delta_right = abs(this->running_check_right - this->right_motor->ticks);
        long delta_left = abs(this->running_check_left - this->left_motor->ticks);

        if((delta_left > 3) || (delta_right > 3))
            this->last_position_update = millis();
        
        this->running_check_right = this->right_motor->ticks;
        this->running_check_left  = this->left_motor->ticks;

        this->IS_RUNNING = ((millis() - this->last_position_update) < this->inactive_delay);
    }
}

float Rolling_Basis::trajectory_theta_calculator(float target_x, float target_y)
{
    /*
    float x_error = target_x - this->X;
    float y_error = target_y - this->Y;
    float cmd_theta;

    // Extremum rotation
    if (x_error == 0 && y_error > 0)
        cmd_theta = PI / 2.0;
    else if (x_error == 0 && y_error < 0)
        cmd_theta = -(PI / 2.0);
    
    // If don't need to rotate
    if (y_error == 0)
        cmd_theta = this->THETA;
    else
        cmd_theta = atan2(y_error, x_error) - this->THETA;

    //*cmd_theta = (*cmd_theta > PI || *cmd_theta < -PI) ? fmod(*cmd_theta, PI) : *cmd_theta; // Noah a eu la merveilleuse idée d'employé un ?
    //cmd_theta = (cmd_theta > PI || cmd_theta < -PI) ? -fmod(cmd_theta, PI) : cmd_theta;
    return cmd_theta;
    */

    float delta_x = fabs(fabs(target_x) - fabs(this->X));
    float delta_y = fabs(fabs(target_y) - fabs(this->Y));
    float cmd_theta;

    // Extremum rotation
    // delta y null
    if (delta_y == 0){
        if (target_x > this->X)
            cmd_theta = 0.0;
        else if (target_x < this->X)
            cmd_theta = PI;
    }
    // delta x null
    else if (delta_x == 0)
    {
        if (target_y > this->Y)
            cmd_theta = PI / 2.0;
        else if (target_x < this->X)
            cmd_theta = -(PI / 2.0);
    }

    // Any cases
    else{
        float adelta_x = target_x - this->X;
        float adelta_y = target_y - this->Y;

        float target_angle = atan2(adelta_y, adelta_x);
        cmd_theta = fmod((target_angle - this->THETA + PI), (2 * PI)) - PI;
    }
    return cmd_theta;
}

float Rolling_Basis::trajectory_distance_calculator(float target_x, float target_y)
{
    /*
    float x_error = target_x - this->X;
    float y_error = target_y - this->Y;
    float cmd_dist;

    cmd_dist = sqrt((x_error * x_error) + (y_error * y_error));
    return cmd_dist;
    */
    // No negative dist
    float delta_x = fabs(fabs(target_x) - fabs(this->X));
    float delta_y = fabs(fabs(target_y) - fabs(this->Y));
    float cmd_dist;

    cmd_dist = sqrt((delta_x * delta_x) + (delta_y * delta_y));

    return cmd_dist;
}

float Rolling_Basis::delta_theta_calculator(float target_theta){
    float current_theta = fmod(this->THETA, 2.0 * PI);
    float delta_theta = abs(abs(current_theta) - abs(this->THETA));

    short sign = 1;
    // W peut etre > au lieu de '<'
    if (target_theta < current_theta)
        sign = -1;
    
    return sign * delta_theta;
}

void Rolling_Basis::theta_to_ticks(float theta_to_rotate, long *ticks, short *r_sign, short *l_sign){
    float distance = (theta_to_rotate * this->radius());
    *ticks = abs((this->encoder_resolution / this->wheel_perimeter()) * distance);

    if (theta_to_rotate > 0.0)
    {
        *r_sign = 1;
        *l_sign = -1;
    }
    else
    {
        *r_sign = -1;
        *l_sign = 1;
    }
}

void Rolling_Basis::distance_to_ticks(float distance, long *ticks, short *r_sign, short *l_sign){
    *ticks = (this->encoder_resolution / this->wheel_perimeter()) * distance;

    if (*ticks > 0)
    {
        *r_sign = 1;
        *l_sign = 1;
    }
    else
    {
        *r_sign = -1;
        *l_sign = -1;
    }
}

void Rolling_Basis::ticks_calculator(Action *action)
{
    if (action->mesurement_taker) {
        short left_sign  = 1;
        short right_sign = 1;
        long cmd_ticks   = 0;

        if(action->start_rotation){  
            float cmd_theta = this->trajectory_theta_calculator(action->target_x, action->target_y);
            this->theta_to_ticks(cmd_theta, &cmd_ticks, &right_sign, &left_sign);
        }
        else if(action->move_forward){
            float cmd_dist = this->trajectory_distance_calculator(action->target_x, action->target_y);
            this->distance_to_ticks(cmd_dist, &cmd_ticks, &right_sign, &left_sign);
        }
        else if (action->end_rotation)
        {
            if (action->target_theta != CLASSIC_ANGLE) {
                float cmd_theta = this->delta_theta_calculator(action->target_theta);
                this->theta_to_ticks(cmd_theta, &cmd_ticks, &right_sign, &left_sign);
            }
            else{
                action->start_rotation = false;
                action->move_forward = false;
                action->end_rotation = false;
                action->mesurement_taker = false;
            }
        }
        if (action->start_rotation || action->move_forward || action->end_rotation){
            action->mesurement_taker = false;
            action->ticks = cmd_ticks;
            action->ticks_cursor = 0;
            action->right_sign = right_sign;
            action->left_sign = left_sign;
            action->right_ref = this->right_motor->ticks;
            action->left_ref = this->left_motor->ticks;
        }
    }
}

void Rolling_Basis::action_handle(Action *current_action) {
    this->ticks_calculator(current_action);

    if (current_action->start_rotation) 
        this->rotation_supervisor(current_action);

    else if (current_action->move_forward)
        this->move_forward_supervisor(current_action);

    else if (current_action->end_rotation)
        this->rotation_supervisor(current_action);

    bool action_running = current_action->start_rotation || current_action->move_forward || current_action->end_rotation;
    if (action_running && !(this->IS_RUNNING)){
        this->left_motor->_threshold_pwm_value  += 200;
        this->right_motor->_threshold_pwm_value += 200;
    }
    else{
        this->left_motor->_threshold_pwm_value  = 0;
        this->right_motor->_threshold_pwm_value = 0;
    }
    motors_controller(current_action);
}

void Rolling_Basis::keep_current_position(long right_ticks, long left_ticks)
{
    double deltaT = this->delta_time_calculator(this->prevT);

    this->right_motor->handle(deltaT, right_ticks, this->max_pwm);
    this->left_motor->handle(deltaT, left_ticks, this->max_pwm);
}

void Rolling_Basis::motors_controller(Action *action){
    double deltaT = this->delta_time_calculator(this->prevT);

    long right_ticks;
    long left_ticks;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        right_ticks = this->right_motor->ticks;
        left_ticks = this->left_motor->ticks;
    }
    long right_error = action->ticks_cursor - abs(right_ticks - action->right_ref);
    long left_error  = action->ticks_cursor - abs(left_ticks  - action->left_ref);

    // Do we need to correct the trajectory ?
    if (abs(right_error) <= this->corrector_error_auth && abs(left_error) <= this->corrector_error_auth)
    {
        if (abs(action->ticks_cursor - action->ticks) < action->error_precision)
            action->ticks_cursor = action->ticks;
        else
            action->ticks_cursor += this->corrector_error_auth;

        if (action->ticks_cursor > action->ticks)
            action->ticks_cursor = action->ticks;
    }

    /*
    if (abs(right_error) < action->error_precision && abs(left_error) < action->error_precision)
    {
        if (abs(action->ticks_cursor - action->ticks) < this->corrector_error_auth)
            action->ticks_cursor = action->ticks;
        else
            action->ticks_cursor += this->corrector_error_auth;

        if (action->ticks_cursor > action->ticks)
            action->ticks_cursor = action->ticks;
    }*/

    this->right_motor->handle(deltaT, (action->right_ref + action->right_sign * action->ticks_cursor), this->max_pwm);
    this->left_motor->handle( deltaT, (action->left_ref  + action->left_sign * action->ticks_cursor ), this->max_pwm);
}

void Rolling_Basis::move_forward_supervisor(Action *action){
    long right_ticks;
    long left_ticks;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        right_ticks = this->right_motor->ticks;
        left_ticks = this->left_motor->ticks;
    }
    long right_error = abs(action->right_ref + action->right_sign * action->ticks) - abs(right_ticks);
    long left_error = abs(action->left_ref + action->left_sign * action->ticks) - abs(left_ticks);

    // Check if the robot is near the target position
    if ((long)action->error_precision >= right_error && (long)action->error_precision >= left_error)
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

void Rolling_Basis::rotation_supervisor(Action *action){
    long right_ticks;
    long left_ticks;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        right_ticks = this->right_motor->ticks;
        left_ticks = this->left_motor->ticks;
    }
    long right_error = abs(action->right_ref + action->right_sign * action->ticks) - abs(right_ticks);
    long left_error = abs(action->left_ref + action->left_sign * action->ticks) - abs(left_ticks);

    // Check if the robot is near the target position
    if ((long)action->error_precision >= right_error && (long)action->error_precision >= left_error)
        (action->end_movement_cpt)++;

    // Checks if the robot has been in the right position long enough to move to the next action
    if (action->end_movement_cpt >= action->end_movement_presicion)
    {
        if(action->start_rotation)
        {
            action->start_rotation = false;
            action->move_forward = true;
            action->end_rotation = false;
            action->mesurement_taker = true;
        }
        else if (action->end_rotation)
        {
            action->start_rotation = false;
            action->move_forward = false;
            action->end_rotation = false;
            action->mesurement_taker = false;
        }
        action->end_movement_cpt = 0;
    }
}
