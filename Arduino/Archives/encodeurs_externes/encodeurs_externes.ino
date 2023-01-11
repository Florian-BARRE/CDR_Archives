byte pin_tick_a = 2;

unsigned long ticks_a = 0;

void Interruption_Tick_a()  { ticks_a++; }
void setup() {
  Serial.begin(9600);
  pinMode(pin_tick_a, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pin_tick_a) , Interruption_Tick_a , RISING);
}

void loop() {
  
  Serial.println((ticks_a/1024.0f)*6*2*3.1415);

}
