// Pins define
#define pin_motor_left_f 8
#define pin_motor_left_b 7
#define pin_motor_left_pwm 5  // PWM pin only !
#define pin_motor_left_tick 2 // AttachInterrupt pin only !

#define pin_motor_right_f 9
#define pin_motor_right_b 10
#define pin_motor_right_pwm 6  // PWM pin only !
#define pin_motor_right_tick 3 // AttachInterrupt pin only !

// DEBUG mode
// 0 = rien
// 1 = traceur série 
// 2 = moniteur série
byte debug_mode = 0;

// States variables of motors
// Nb ticks
unsigned long motor_left_nb_ticks = 0;
unsigned long motor_right_nb_ticks = 0;
// current RPM
float motor_left_current_rpm  = 0.0;
float motor_right_current_rpm = 0.0;
// current PWM
int motor_left_current_pwm_value  = 0;
byte motor_right_current_pwm_value = 0;

// Timer variables
unsigned long last_rpm_calculator_refresh = millis();
unsigned long last_pwm_value_update       = millis();

// Init pin INPUT / OUTPUT
// Interruptions
void interruption_tick_left()  { motor_left_nb_ticks++;  }
void interruption_tick_right() { motor_right_nb_ticks++; }
// Setup
void motors_init(){
  pinMode(pin_motor_left_f, OUTPUT);
  pinMode(pin_motor_left_b, OUTPUT);
  pinMode(pin_motor_left_pwm, OUTPUT);
  pinMode(pin_motor_left_tick, INPUT);

  pinMode(pin_motor_right_f, OUTPUT);
  pinMode(pin_motor_right_b, OUTPUT);
  pinMode(pin_motor_right_pwm, OUTPUT);
  pinMode(pin_motor_right_tick, INPUT);

  attachInterrupt(digitalPinToInterrupt(pin_motor_left_tick) , interruption_tick_left , RISING);
  attachInterrupt(digitalPinToInterrupt(pin_motor_right_tick), interruption_tick_right, RISING);
}

// State reader function
void calculate_rpm(word echantillon_duration){
  /*
   * Calcul RPM:
   * 1 tour = 1024 ticks
   * rpm = nb de tours * nb de fois que l'échantillon se retrouve dans 1 minute
   * rpm = (nb de ticks / 1024) * (60000 / durée de l'échantillon)
   */
  if((millis() - last_rpm_calculator_refresh) >= echantillon_duration){  
    motor_left_current_rpm  = ((float) motor_left_nb_ticks  / 1024) * (60000 / echantillon_duration);
    motor_right_current_rpm = ((float) motor_right_nb_ticks / 1024) * (60000 / echantillon_duration);

    last_rpm_calculator_refresh = millis();

    if(debug_mode == 1){
      
      Serial.println(motor_right_current_pwm_value);
      Serial.print(",");
      Serial.println(motor_right_current_rpm);
      
      /*
      Serial.print(motor_left_current_rpm);
      Serial.print(",");
      Serial.println(motor_right_current_rpm);
      */
    }
    else if (debug_mode == 2){
      Serial.print("Left motor: ");
      Serial.print(motor_left_current_rpm);
      Serial.println("rpm");
    
      Serial.print("Right motor: ");
      Serial.print(motor_right_current_rpm);
      Serial.println("rpm");   
    }
    Serial.println(motor_left_nb_ticks);
      Serial.print(",");
      Serial.println(motor_right_nb_ticks);
        
    motor_left_nb_ticks  = 0;
    motor_right_nb_ticks = 0;
  }
}

void Read_Channels(word wait){
  long start_millis = millis();
  while((millis() - start_millis) <= wait){
    Serial.print(digitalRead(pin_motor_left_tick));
    Serial.print(",");
    Serial.print(digitalRead(pin_motor_right_tick));
    Serial.println();
  }
}

// Basics orders

// Forward
void Forward(byte pwm_left, byte pwm_right){
  // Left
  analogWrite(pin_motor_left_pwm, pwm_left); 
  digitalWrite(pin_motor_left_f, HIGH);
  digitalWrite(pin_motor_left_b, LOW);

  // Right
  analogWrite(pin_motor_right_pwm, pwm_right); 
  digitalWrite(pin_motor_right_f, HIGH);
  digitalWrite(pin_motor_right_b, LOW);
}

// Backward
void Backward(byte pwm_left, byte pwm_right){
  // Left
  analogWrite(pin_motor_left_pwm, pwm_left); 
  digitalWrite(pin_motor_left_f, LOW);
  digitalWrite(pin_motor_left_b, HIGH);

  // Right
  analogWrite(pin_motor_right_pwm, pwm_right); 
  digitalWrite(pin_motor_right_f, LOW);
  digitalWrite(pin_motor_right_b, HIGH);
}

// Stop
void Stop(){
  // Left
  digitalWrite(pin_motor_left_f, LOW);
  digitalWrite(pin_motor_left_b, LOW);

  // Right
  digitalWrite(pin_motor_right_f, LOW);
  digitalWrite(pin_motor_right_b, LOW);
}

// Advanced orders

// Forward
void calculate_pwm(float target_rpm, byte update_requency){
  if((millis() - last_pwm_value_update) >= update_requency){ 
    // Left 
    if      (target_rpm > motor_left_current_rpm && motor_left_current_pwm_value < 255){ motor_left_current_pwm_value++; }
    else if (target_rpm < motor_left_current_rpm && motor_left_current_pwm_value > 0  ){ motor_left_current_pwm_value--; }

    // Right
    if      (target_rpm > motor_right_current_rpm && motor_right_current_pwm_value < 255){ motor_right_current_pwm_value++; }
    else if (target_rpm < motor_right_current_rpm && motor_right_current_pwm_value > 0  ){ motor_right_current_pwm_value--; }
    
    last_pwm_value_update = millis();
  }
}

void forward_rpm(float target_rpm, byte refresh_rpms, byte refresh_pwm_correction){
  calculate_rpm(refresh_rpms);
  calculate_pwm(target_rpm, refresh_pwm_correction);
  Forward(motor_left_current_pwm_value, motor_right_current_pwm_value);
}