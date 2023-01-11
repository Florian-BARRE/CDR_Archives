class Motor {

    private:
    
        // Pins Motor
        byte _pin_forward;
        byte _pin_backward;
        byte _pin_pwm;  // PWM pin only !
        byte _pin_tick; // AttachInterrupt pin only !  

        // Timer variables
        unsigned long _last_rpm_calculator_update = millis();
        unsigned long _last_pwm_value_update      = millis();

    public: 
        // Tick interruption
        volatile unsigned long _nb_ticks = 0;

        static void ISRFunc(void* properties) {
          static_cast<Motor*>(properties)->_interruption_tick(); 
        }
        void _interruption_tick(){
            _nb_ticks++;
        }

        // Motor states
        float current_rpm  = 0.0;
        byte current_pwm_value = 0;

        // Natural constructor
        Motor(byte pin_forward, byte pin_backward, byte pin_pwm, byte pin_tick){
          Serial.println("DEBUT");
          Serial.println(_pin_forward);
          Serial.println(_pin_backward);
          Serial.println(_pin_pwm);
          Serial.println(_pin_tick);
          Serial.println("DEBUT");
            _pin_forward  = pin_forward ;
            _pin_backward = pin_backward;
            _pin_pwm      = pin_pwm     ; // PWM pin only !
            _pin_tick     = pin_tick    ; // AttachInterrupt pin only ! 
        }

        void init(){
            pinMode(_pin_forward, OUTPUT);
            pinMode(_pin_backward, OUTPUT);
            pinMode(_pin_pwm, OUTPUT);
            pinMode(_pin_tick, INPUT);
            
            attachInterrupt(digitalPinToInterrupt(_pin_tick) , &Motor::ISRFunc, RISING);
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
          Serial.println("A");
          Serial.println(_pin_pwm);
          Serial.println(pwm);
          Serial.println(_pin_forward);
          Serial.println(_pin_backward);
          
            analogWrite(_pin_pwm, pwm); 
            digitalWrite(_pin_forward, HIGH);
            digitalWrite(_pin_backward, LOW);
            Serial.println("B");
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
            forward(current_rpm);
        }
        
};
