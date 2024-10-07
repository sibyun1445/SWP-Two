// Arduino pin assignment
#define PIN_LED 9
#define PIN_TRIG 12   // sonar sensor TRIGGER
#define PIN_ECHO 13   // sonar sensor ECHO

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25      // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100.0   // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300.0   // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL) // coefficient to convert duration to distance

int ledBrightness = 0; // LED 밝기를 저장할 변수

void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);  // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);   // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar 
  
  // initialize serial port
  Serial.begin(57600);
}

void loop() {
  float distance = USS_measure(PIN_TRIG, PIN_ECHO); // read distance

  // LED 밝기 설정
  if ((distance <= _DIST_MIN) || (distance >= _DIST_MAX)) {
      ledBrightness = 255; // 최대 밝기
  } else if (distance <= 150.0 || distance >= 250.0) {
      ledBrightness = 128; // 중간 밝기
  } else if (distance == 200.0) { // 원하는 거리
      ledBrightness = 0; // LED 꺼짐
  } else {
      ledBrightness = 0; // 그 외 거리에서는 LED 꺼짐
  }

  analogWrite(PIN_LED, ledBrightness); // LED 밝기 설정

  // output the distance to the serial port
  Serial.print("Min:");        Serial.print(_DIST_MIN);
  Serial.print(",distance:");  Serial.print(distance);
  Serial.print(",Max:");       Serial.print(_DIST_MAX);
  Serial.println("");
  
  // wait until next sampling time.
  delay(INTERVAL);
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
