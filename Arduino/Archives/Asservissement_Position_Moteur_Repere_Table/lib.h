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
      byte _pwm_dif;
      
      // Variables 
      float _error_prev = 0;
      float _error_integral = 0;
        
    public: 
        // Attach interrupt variable
        volatile long ticks = 0; 

        // Natural constructor
        Motor(byte pin_forward, byte pin_backward, byte pin_pwm, byte pin_enca, byte pin_encb, float kp, float kd, float ki, byte pwm_dif){
            _pin_forward   = pin_forward;
            _pin_backward  = pin_backward;
            _pin_pwm       = pin_pwm;  // PWM pin only !
            _pin_enca      = pin_enca; // AttachInterrupt pin only ! 
            _pin_encb      = pin_encb; // AttachInterrupt pin only !  

            _kp = kp;
            _kd = kd;
            _ki = ki;
            _pwm_dif = pwm_dif;
        }

        void init(){
            pinMode(_pin_forward, OUTPUT);
            pinMode(_pin_backward, OUTPUT);
            pinMode(_pin_pwm, OUTPUT);

            pinMode(_pin_enca, INPUT);
            pinMode(_pin_encb, INPUT);
        }
        
        void set_motor(int8_t dir, byte pwmVal, byte pwm, byte in1, byte in2){
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

        void handle(float delta_time, long target_pos, byte max_speed){
            long fix_ticks = 0;
              ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
              {
                  fix_ticks = ticks;
              }
          
              // Calculate error
              int error = fix_ticks - target_pos;
          
              // Calculate derivative
              double dedt = (error - _error_prev) / delta_time;
          
              // Calculate integral
              _error_integral = _error_integral + (error * delta_time);
          
              // Control signal
              float u = _kp * error + _kd * dedt + _ki * _error_integral;
          
              // Motor power
              float power = fabs(u) - _pwm_dif;
              if (power > max_speed)
                  power = max_speed;
              if(power < 0)
                power = 0;
          
              // Motor direction
              int8_t direction = 1;
              if (u < 0)
                  direction = -1;
          
              // Set the correct motor commande
              set_motor(direction, power, _pin_pwm, _pin_forward, _pin_backward);
          
              // Save error
              _error_prev = error;
          
              Serial.print(fix_ticks);
              Serial.print(" ");
              Serial.print(target_pos);
        }       
};
