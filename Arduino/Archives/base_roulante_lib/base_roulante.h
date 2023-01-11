#include "motor.h"

class Base_Roulante{

    private:
        // Motor instantiate index
        byte _motor_instantiate_index = 0;
        #define motors_size 2
        Motor[] motors = new Motor[motors_size];
  
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
            if(motors_size > _motor_instantiate_index){
               Serial.println("MOTOR1");
               motors[_motor_instantiate_index] = Motor(pin_forward, pin_backward, pin_pwm, pin_tick);
            }
            _motor_instantiate_index++;
        }

        // Setup Motors
        void init(){
            for(byte k=0; (k<_motor_instantiate_index && motors_size > _motor_instantiate_index); k++){
              motors[k].init();
            }
        }

        // Deplacements functions

        // Classics
        // Forward 
        void forward(byte pwm){
          for(byte k=0; (k<_motor_instantiate_index && motors_size > _motor_instantiate_index); k++){
              motors[k].forward(pwm);
          }
           
        }
    /*
        // Backward 
        void backward(byte pwm){
            _left_motor.backward(pwm); 
            _right_motor.backward(pwm);
        }

        // Stop 
        void backward(){
            _left_motor.stop(); 
            _right_motor.stop();
        }*/
};
