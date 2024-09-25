#define PIN_LED 7
int n=0;
void setup() {
  pinMode(PIN_LED, OUTPUT);
}

void loop() {
  digitalWrite(PIN_LED,LOW);
  delay(1000);
  n=0;
  while(n<5) {
    digitalWrite(PIN_LED, 1);
    delay(100);
    digitalWrite(PIN_LED, 0);
    delay(100);
    n+=1;
  }
  digitalWrite(PIN_LED,HIGH);

   while(1) {
    // 아무 작업도 하지 않고 무한 루프
  }
}
