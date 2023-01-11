class Motor {
    private:
        // Pins Motor
        const byte _pin_forward;
        const byte _pin_backward;
        const byte _pin_pwm;  // PWM pin only !
        const byte _pin_tick; // AttachInterrupt pin only !  

        // Ticks
        unsigned long _nb_ticks = 0;
        _interruption_tick(){
            _nb_ticks++;
        }

        // Timer variables
        unsigned long _last_rpm_calculator_update = millis();
        unsigned long _last_pwm_value_update      = millis();

    public: 
        unsigned float current_rpm  = 0.0;
        byte current_pwm_value = 0;

        Motor(const byte pin_forward, const byte pin_backward, const byte pin_pwm, const byte pin_tick){
            _pin_forward  = pin_forward ;
            _pin_backward = pin_backward;
            _pin_pwm      = pin_pwm     ; // PWM pin only !
            _pin_tick     = pin_tick    ; // AttachInterrupt pin only ! 
        }

        // Motor Setup
        void setup(){
            pinMode(pin_forward, OUTPUT);
            pinMode(pin_backward, OUTPUT);
            pinMode(pin_pwm, OUTPUT);
            pinMode(pin_tick, INPUT);
            
            attachInterrupt(digitalPinToInterrupt(pin_tick) , _interruption_tick , RISING);
        }

        // Update / Calculate new current RPM 
        void calculate_rpm(word echantillon_duration){
            /*
            * Calcul RPM:
            * 1 tour = 1024 ticks
            * rpm = nb de tours * nb de fois que l'échantillon se retrouve dans 1 minute
            * rpm = (nb de ticks / 1024) * (60000 / durée de l'échantillon)
            */
            if((millis() - _last_rpm_calculator_update) >= echantillon_duration){  
                current_rpm  = ((float) _nb_ticks  / 1024) * (60000 / echantillon_duration);
                // Re init
                _last_rpm_calculator_update = millis();
                _nb_ticks = 0;
            }
        }

        // Update / Calculate new current PWM 
        void calculate_pwm(float target_rpm, byte update_requency){
            
            if((millis() - _last_pwm_value_update) >= update_requency){ 
                if      (target_rpm > current_rpm && current_rpm < 255){ current_pwm_value++; }
                else if (target_rpm < current_rpm && current_rpm > 0  ){ current_pwm_value--; }
                // Re init
                _last_pwm_value_update = millis();
            }
        }

        // Deplacements functions

        // Classics
        // Forward
        void forward(byte pwm){
            analogWrite(_pin_pwm, pwm); 
            digitalWrite(_pin_forward, HIGH);
            digitalWrite(_pin_backward, LOW);
        }
        // Backward
        void backward(byte pwm){
            analogWrite(_pin_pwm, pwm); 
            digitalWrite(_pin_forward, LOW);
            digitalWrite(_pin_backward, HIGH);
        }
        // Stop
        void stop(){
            current_pwm_value = 0;
            analogWrite(_pin_pwm, 0); 
            digitalWrite(_pin_forward, LOW);
            digitalWrite(_pin_backward, LOW);
        }

        // Advanced
        // Forward
        void forward_RPM(float target_rpm, byte refresh_rpms, byte refresh_pwm_correction){
            calculate_rpm(refresh_rpms);
            calculate_pwm(target_rpm, refresh_pwm_correction);
            forward(motor_left_current_pwm_value, motor_right_current_pwm_value);
        }
        
}