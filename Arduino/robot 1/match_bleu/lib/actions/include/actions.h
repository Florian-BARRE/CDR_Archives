#pragma once
#include <Arduino.h>
#define CLASSIC_ANGLE 1234.1234
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

    long ticks;
    long ticks_cursor;

    long right_ref;
    long left_ref;
    short right_sign;
    short left_sign;
};

// Action creation
Action create_action(float obj_x, float obj_y, unsigned int precision_cpt, unsigned int error_auth, float obj_theta = CLASSIC_ANGLE);

void display_action(Action *act);