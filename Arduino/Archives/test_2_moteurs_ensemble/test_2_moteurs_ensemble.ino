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

#define entraxe 275
#define diametre 61

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
}

void loop() {
  Forward(100);
  Read_Channels(1000);
  Stop();
  Read_Channels(1000);
  
  Backward(50);
  Read_Channels(1000);
  Stop();
  Read_Channels(1000);
}

void Read_Channels(word wait){
  word count = 0;
  while(count < wait){
    Serial.print(digitalRead(pin_motor_left_tick));
    Serial.print(",");
    Serial.print(digitalRead(pin_motor_right_tick));
    Serial.println();
    count++;
    delay(1);
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
