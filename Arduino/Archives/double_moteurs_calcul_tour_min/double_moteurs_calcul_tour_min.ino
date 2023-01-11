/*
 * LEFT
 * Moteur A:
 * Sens F -> Pin 2
 * Sens B -> Pin 4
 * PWM    -> Pin 3
 * Tick   -> Pin 5
 * 
 * RIGHT
 * Moteur B:
 * Sens F -> Pin 7
 * Sens B -> Pin 8
 * PWM    -> Pin 9
 * Tick   -> Pin 10
 * 
 */
#define pin_motor_left_f 8
#define pin_motor_left_b 7
#define pin_motor_left_pwm 5
#define pin_motor_left_tick 2

#define pin_motor_right_f 9
#define pin_motor_right_b 10
#define pin_motor_right_pwm 6
#define pin_motor_right_tick 3

void setup() {
  Serial.begin(2000000);
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
  Forward(100);
}

unsigned long motor_left_nb_ticks = 0;
unsigned long motor_right_nb_ticks = 0;
unsigned long last_rpm_calculator_refresh = millis();

void loop() {
  Rpm_calcul(50);
}



// Calcul RPM
// Interruptions
void Interruption_Tick_Left() {
    motor_left_nb_ticks++;
}
void Interruption_Tick_Right() {
    motor_right_nb_ticks++;
}
// Display RPM
void Rpm_calcul(word echantillon_duration){
  /*
   * Calcul RPM:
   * 1 tour = 1024 ticks
   * rpm = nb de tours * nb de fois que l'échantillon se retrouve dans 1 minute
   * rpm = (nb de ticks / 1024) * (60000 / durée de l'échantillon)
   */
  if((millis() - last_rpm_calculator_refresh) >= echantillon_duration){  
    float motor_left_rpm  = ((float) motor_left_nb_ticks  / 1024) * (60000 / echantillon_duration);
    float motor_right_rpm = ((float) motor_right_nb_ticks / 1024) * (60000 / echantillon_duration);

    last_rpm_calculator_refresh = millis();

    /*
    Serial.print("Left motor: ");
    Serial.print(motor_left_rpm);
    Serial.println("rpm");
    
    Serial.print("Right motor: ");
    Serial.print(motor_right_rpm);
    Serial.println("rpm");   
    */

    
    Serial.print(motor_left_rpm);
    Serial.print(",");
    Serial.println(motor_right_rpm);
    
    
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

void Forward(byte pwm){
  // Left
  analogWrite(pin_motor_left_pwm, pwm); 
  digitalWrite(pin_motor_left_f, HIGH);
  digitalWrite(pin_motor_left_b, LOW);

  // Right
  analogWrite(pin_motor_right_pwm, pwm); 
  digitalWrite(pin_motor_right_f, HIGH);
  digitalWrite(pin_motor_right_b, LOW);
}

void Backward(byte pwm){
  // Left
  analogWrite(pin_motor_left_pwm, pwm); 
  digitalWrite(pin_motor_left_f, LOW);
  digitalWrite(pin_motor_left_b, HIGH);

  // Right
  analogWrite(pin_motor_right_pwm, pwm); 
  digitalWrite(pin_motor_right_f, LOW);
  digitalWrite(pin_motor_right_b, HIGH);
}

void Stop(){
  // Left
  digitalWrite(pin_motor_left_f, LOW);
  digitalWrite(pin_motor_left_b, LOW);

  // Right
  digitalWrite(pin_motor_right_f, LOW);
  digitalWrite(pin_motor_right_b, LOW);
}
