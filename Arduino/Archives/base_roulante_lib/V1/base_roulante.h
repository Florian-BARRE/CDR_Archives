#include "motor.h"

class Base_Roulante{

    private:
        // Motor instantiate index
        byte _motor_instantiate_index = 0;
        // Left Motor
        Motor &_left_motor;
        // Right Motor
        Motor &_right_motor;

        // Properties
        /*
        entraxe etc etc
        */

    public:
        // Natural Constructor
        Base_Roulante(){
        /*
        entraxe etc etc
        */
        }

        // Add motor
        void add_motor(byte pin_forward, byte pin_backward, byte pin_pwm, byte pin_tick){
            if(_motor_instantiate_index == 0){
              Serial.println("MOTOR1");
                _left_motor = Motor(pin_forward, pin_backward, pin_pwm, pin_tick);
            }
            else if(_motor_instantiate_index == 1){
              Serial.println("MOTOR2");
                _right_motor = Motor(pin_forward, pin_backward, pin_pwm, pin_tick);
            }
            _motor_instantiate_index++;
        }

        // Setup Motors
        void init(){
            if(_motor_instantiate_index >= 1){_left_motor.init(); }
            if(_motor_instantiate_index >= 2){_right_motor.init();}
        }

        // Deplacements functions

        // Classics
        // Forward 
        void forward(byte pwm){
            _left_motor.forward(pwm); 
            _right_motor.forward(pwm);
        }

        // Backward 
        void backward(byte pwm){
            _left_motor.backward(pwm); 
            _right_motor.backward(pwm);
        }

        // Stop 
        void backward(){
            _left_motor.stop(); 
            _right_motor.stop();
        }
};
