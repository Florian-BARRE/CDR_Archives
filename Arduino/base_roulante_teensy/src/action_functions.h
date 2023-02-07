#include <Arduino.h>

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

// Action creation
Action create_action(float obj_x, float obj_y, float obj_theta)
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

    // Action new_action = {obj_x, obj_y, obj_theta, true, false, false, true, 0, precision_cpt, error_auth, 0, 0};
    return new_action;
}

void display_action(Action *act)
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