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
byte motor_left_current_pwm_value  = 0;
byte motor_right_current_pwm_value = 0;

// Timer variables
unsigned long last_rpm_calculator_refresh = millis();
unsigned long last_pwm_value_update       = millis();


// Init pin INPUT / OUTPUT
// Interruptions
void Interruption_Tick_Left()  { motor_left_nb_ticks++;  }
void Interruption_Tick_Right() { motor_right_nb_ticks++; }
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

  attachInterrupt(digitalPinToInterrupt(pin_motor_left_tick) , Interruption_Tick_Left , RISING);
  attachInterrupt(digitalPinToInterrupt(pin_motor_right_tick), Interruption_Tick_Right, RISING);
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
      
      
      Serial.print(motor_left_current_rpm);
      Serial.print(",");
      Serial.println(motor_right_current_rpm);
      
    }
    else if (debug_mode == 2){
      Serial.print("Left motor: ");
      Serial.print(motor_left_current_rpm);
      Serial.println("rpm");
    
      Serial.print("Right motor: ");
      Serial.print(motor_right_current_rpm);
      Serial.println("rpm");   
    }   
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
void Forward_r(byte pwm){
  analogWrite(pin_motor_right_pwm, pwm); 
  digitalWrite(pin_motor_right_f, HIGH);
  digitalWrite(pin_motor_right_b, LOW);
}
void Forward_l(byte pwm){
  analogWrite(pin_motor_left_pwm, pwm); 
  digitalWrite(pin_motor_left_f, HIGH);
  digitalWrite(pin_motor_left_b, LOW);
}
void Forward(byte pwm){
  Forward_r(pwm);
  Forward_l(pwm);
}

// Backward
void Backward_r(byte pwm){
  analogWrite(pin_motor_right_pwm, pwm); 
  digitalWrite(pin_motor_right_b, HIGH);
  digitalWrite(pin_motor_right_f, LOW);
}
void Backward_l(byte pwm){
  analogWrite(pin_motor_left_pwm, pwm); 
  digitalWrite(pin_motor_left_b, HIGH);
  digitalWrite(pin_motor_left_f, LOW);
}
void Backward(byte pwm){
  Backward_r(pwm);
  Backward_l(pwm);
}

// Stop
void Stop_r(){
  digitalWrite(pin_motor_right_f, LOW);
  digitalWrite(pin_motor_right_b, LOW);
}
void Stop_l(){
  digitalWrite(pin_motor_left_f, LOW);
  digitalWrite(pin_motor_left_b, LOW);
}
void Stop(){
  Stop_r();
  Stop_l();
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

void Forward_rpm(float target_rpm, byte refresh_rpms, byte refresh_pwm_correction){
  calculate_rpm(refresh_rpms);
  calculate_pwm(target_rpm, refresh_pwm_correction);
  Forward_r(motor_right_current_pwm_value);
  Forward_l(motor_left_current_pwm_value);
  //Forward(motor_left_current_pwm_value, motor_right_current_pwm_value);
}

void Forward_ticks(unsigned long ticks, byte pwm){
  motor_left_current_pwm_value  = pwm;
 // motor_right_current_pwm_value = pwm;
  motor_left_nb_ticks = 0;
  //motor_right_nb_ticks = 0;
  
  
  while(motor_left_nb_ticks < ticks){
    if(motor_left_nb_ticks < ticks){
      Forward_l(pwm);
    }
    else{
      Stop_l();
    }
  }
}
