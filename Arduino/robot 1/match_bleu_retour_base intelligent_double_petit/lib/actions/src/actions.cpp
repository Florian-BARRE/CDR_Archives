#include "actions.h"

Action create_action(float obj_x, float obj_y, unsigned int precision_cpt, unsigned int error_auth, float obj_theta = CLASSIC_ANGLE)
{
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
    new_action.ticks = 0;
    new_action.ticks_cursor = 0;
    new_action.right_sign = 1;
    new_action.left_sign = 1;

    // Action new_action = {obj_x, obj_y, obj_theta, true, false, false, true, 0, precision_cpt, error_auth, 0, 0};
    return new_action;
}

void display_action(Action *act)
{
    Serial.println();
    Serial.print("target_x : ");
    Serial.println(act->target_x);
    Serial.print("target_y : ");
    Serial.println(act->target_y);
    Serial.print("target_theta : ");
    Serial.println(act->target_theta);
    Serial.print("start rotation : ");
    Serial.println(act->start_rotation);
    Serial.print("move_forward : ");
    Serial.println(act->move_forward);
    Serial.print("end rotation : ");
    Serial.println(act->end_rotation);
    Serial.print("mesurement_taker : ");
    Serial.println(act->mesurement_taker);
    Serial.print("end_movement_cpt : ");
    Serial.println(act->end_movement_cpt);
    Serial.print("end_movement_presicion : ");
    Serial.println(act->end_movement_presicion);
    Serial.print("error_precision : ");
    Serial.println(act->error_precision);
    Serial.print("ticks : ");
    Serial.println(act->ticks);
    Serial.print("ticks_cursor : ");
    Serial.println(act->ticks_cursor);
    Serial.print("right_sign : ");
    Serial.println(act->right_sign);
    Serial.print("left_sign : ");
    Serial.println(act->left_sign);
    Serial.println();
}