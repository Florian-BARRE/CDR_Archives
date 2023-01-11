#include <util/atomic.h> 
class Motor {
    private:
        // Pins Motor
        byte _pin_forward;
        byte _pin_backward;
        byte _pin_pwm;  // PWM pin only !
        byte _pin_enca; // AttachInterrupt pin only ! 
        byte _pin_encb; // AttachInterrupt pin only !  

        // PID constantes
        float _kp;
        float _kd;
        float _ki;

        // Variables
        volatile int _posi = 0; 
        float _error_prev = 0;
        float _error_integral = 0;

    public: 
        static void ISRFunc_ReadEncoder(void* properties) {
          static_cast<Motor*>(properties)->_read_encoder(); 
        }
        
        void _read_encoder(){
            if(digitalRead(_pin_encb) > 0)  _posi++;
            else                            _posi--;
        }

        // Natural constructor
        Motor(byte pin_forward, byte pin_backward, byte pin_pwm, byte pin_enca, byte pin_encb, float kp, float kd, float ki){
            byte _pin_forward   = pin_forward;
            byte _pin_backward  = pin_backward;
            byte _pin_pwm       = pin_pwm;  // PWM pin only !
            byte _pin_enca      = pin_enca; // AttachInterrupt pin only ! 
            byte _pin_encb      = pin_encb; // AttachInterrupt pin only !  

            _kp = kp;
            _kd = kd;
            _ki = k1;
        }

        void init(){
            pinMode(_pin_forward, OUTPUT);
            pinMode(_pin_backward, OUTPUT);
            pinMode(_pin_pwm, OUTPUT);

            pinMode(_pin_enca, INPUT);
            pinMode(_pin_encb, INPUT);
     
            attachInterrupt(digitalPinToInterrupt(_pin_enca), &Motor::ISRFunc_ReadEncoder, RISING);
        }
        
        void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
            analogWrite(pwm, pwmVal);
            if(dir == 1){
                digitalWrite(in1,HIGH);
                digitalWrite(in2,LOW);
            }
            else if(dir == -1){
                digitalWrite(in1,LOW);
                digitalWrite(in2,HIGH);
            }
            else{
                digitalWrite(in1,LOW);
                digitalWrite(in2,LOW);
            }  
        } 

        void handle(float delta_time, int target_pos){
            int pos = 0;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                pos = _posi;
            }

            // Calculate error 
            int error = pos - target_pos;

            // Calculate derivative
            float dedt = (error - _error_prev)/delta_time;

            // Calculate integral
            _error_integral = _error_integral + (error*delta_time);

            // Control signal
            float u = _kp*error + _kd*dedt + _ki*_error_integral;

            // Motor power
            float power = fabs(u);
            if( power > 255 )  power = 255;

            // Motor direction
            int direction = 1;
            if(u < 0)  direction = -1;

            // Set the correct motor commande
            setMotor(direction, power, pwm, _pin_forward, _pin_backward);

            // Save error
            _error_prev = error;
        }       
};
