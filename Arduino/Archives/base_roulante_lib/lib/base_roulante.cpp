#include "motor.cpp"

class Base_Roulante {

    private:
        // Motor instantiate index
        byte _motor_instantiate_index = 0;
        // Left Motor
        Motor _left_motor;
        // Right Motor
        Motor _right_motor;

        // Properties
        /*
        entraxe etc etc
        */

    public:
        Base_Roulante(){
            // Entreaxe etc etc
        }

        // Add motor
        void add_motor(const byte pin_forward, const byte pin_backward, const byte pin_pwm, const byte pin_tick){
            if(_motor_instantiate_index == 0){
                _left_motor = new Motor(pin_forward, byte pin_backward, byte pin_pwm, byte pin_tick);
            }
            else if(_motor_instantiate_index == 1){
                _right_motor = new Motor(pin_forward, byte pin_backward, byte pin_pwm, byte pin_tick);
            }
            _motor_instantiate_index++;
        }

        // Setup Motors
        void setup(){
            if(_motor_instantiate_index >= 1){_left_motor.setup(); }
            if(_motor_instantiate_index >= 2){_right_motor.setup();}
        }

        

}