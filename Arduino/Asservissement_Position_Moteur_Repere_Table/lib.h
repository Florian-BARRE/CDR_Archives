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
      float _error_prev = 0;
      float _error_integral = 0;
        
    public: 
        // Attach interrupt variable
        volatile long volatile_pos = 0; 

        // Natural constructor
        Motor(byte pin_forward, byte pin_backward, byte pin_pwm, byte pin_enca, byte pin_encb, float kp, float kd, float ki){
            _pin_forward   = pin_forward;
            _pin_backward  = pin_backward;
            _pin_pwm       = pin_pwm;  // PWM pin only !
            _pin_enca      = pin_enca; // AttachInterrupt pin only ! 
            _pin_encb      = pin_encb; // AttachInterrupt pin only !  

            _kp = kp;
            _kd = kd;
            _ki = ki;
        }

        void init(){
            pinMode(_pin_forward, OUTPUT);
            pinMode(_pin_backward, OUTPUT);
            pinMode(_pin_pwm, OUTPUT);

            pinMode(_pin_enca, INPUT);
            pinMode(_pin_encb, INPUT);
        }
        
        void setMotor(int8_t dir, byte pwmVal, byte pwm, byte in1, byte in2){
            analogWrite(pwm, pwmVal);
            if(dir == -1){
                digitalWrite(in1,HIGH);
                digitalWrite(in2,LOW);
            }
            else if(dir == 1){
                digitalWrite(in1,LOW);
                digitalWrite(in2,HIGH);
            }
            else{
                digitalWrite(in1,LOW);
                digitalWrite(in2,LOW);
            }  
        } 

        void handlee(float delta_time, long target_pos, byte max_speed){
            long pos = 0;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                pos = volatile_pos;
            }
            /*
            Serial.print(target_pos);
            Serial.print(" ");
            Serial.print(pos);
            Serial.print(" ");
            */

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
            if( power > max_speed )  power = max_speed;

            // Motor direction
            int8_t direction = 1;
            if(u < 0)  direction = -1;

            // Set the correct motor commande
            setMotor(direction, power, _pin_pwm, _pin_forward, _pin_backward);

            // Save error
            _error_prev = error;
        }       
};
