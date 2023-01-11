byte pin_motor_r_f   = 5;
byte pin_motor_r_b   = 6;
byte pin_motor_r_pwm = 3;

byte pin_channel_A = A0;
byte pin_channel_B = A1;
byte pin_channel_C = A2;

void setup() {
  Serial.begin(2000000);
  pinMode(pin_motor_r_f, OUTPUT);
  pinMode(pin_motor_r_b, OUTPUT);
  pinMode(pin_motor_r_pwm, OUTPUT);
  
}

void loop() {
  //Forward(40);
  Read_channel();

  /*
  Stop();
  delay(1000);
  
  Backward(20);
  while(wait < 2000){
    Read_channel();
    wait++;
  }
  Stop();
  delay(1000);
  */
}

void Read_channel(){
  //Serial.print("Channel A: ");
 
  //Serial.print("Channel B: ");
  //Serial.println(analogRead(pin_channel_B));
  
  //Serial.print("Channel C: ");
  //Serial.println(analogRead(pin_channel_C));


  Serial.print(analogRead(pin_channel_A));
  Serial.print(",");
  Serial.print(analogRead(pin_channel_B));
  Serial.print(",");
  Serial.println(analogRead(pin_channel_C));
}
void Forward(byte pwm){
  analogWrite(pin_motor_r_pwm, pwm); 
  digitalWrite(pin_motor_r_f, HIGH);
  digitalWrite(pin_motor_r_b, LOW);
}

void Backward(byte pwm){
  analogWrite(pin_motor_r_pwm, pwm); 
  digitalWrite(pin_motor_r_f, LOW);
  digitalWrite(pin_motor_r_b, HIGH);
}

void Stop(){
  digitalWrite(pin_motor_r_f, LOW);
  digitalWrite(pin_motor_r_b, LOW);
}
