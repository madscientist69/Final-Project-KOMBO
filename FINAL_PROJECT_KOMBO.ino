#include <tcs3200.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// ----------------------------
// PIN SETUP
// ----------------------------
SoftwareSerial BT(2, 3);  // RX, TX

#define S0_PIN 4
#define S1_PIN 5
#define S2_PIN 6
#define S3_PIN 7
#define OUT_PIN 8
TCS3200 colorSensor(S0_PIN, S1_PIN, S2_PIN, S3_PIN, OUT_PIN);

Servo RServo;
Servo GServo;
Servo WServo;

#define IN1 12
#define IN2 13
#define IN3 A0
#define IN4 A1

// Timing (non-blocking)
unsigned long lastColorRead = 0;
unsigned long colorInterval = 120;  // read sensor every 120 ms

// Store last detected color (to avoid servo jitter)
String lastColor = "NONE";

void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  RServo.attach(9);
  GServo.attach(10);
  WServo.attach(11);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Sensor
  colorSensor.begin();
  colorSensor.calibrate_light();
  delay(1000);
  colorSensor.calibrate_dark();
  delay(1000);
  colorSensor.calibrate();

  Serial.println("READY");
}

// ----------------------------
// MOTOR FUNCTIONS
// ----------------------------
void motorForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void motorBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
void motorLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void motorRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
void motorStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ----------------------------
// SET SERVO POSITION ONLY WHEN COLOR CHANGES
// ----------------------------
void setColorServos(String color) {
  if (color == lastColor) return;   // prevent jitter
  lastColor = color;

  int pos = 45;

  if (color == "RED") {
    RServo.write(pos + 45);
    GServo.write(pos);
    WServo.write(pos);
  }
  else if (color == "GREEN") {
    GServo.write(pos + 45);
    RServo.write(pos);
    WServo.write(pos);
  }
  else if (color == "WHITE") {
    WServo.write(pos + 45);
    RServo.write(pos);
    GServo.write(pos);
  }
  else if (color == "GRAY") {
    RServo.write(pos + 45);
    GServo.write(pos + 45);
    WServo.write(pos + 45);
  }
  else {
    RServo.write(pos);
    GServo.write(pos);
    WServo.write(pos);
  }

  Serial.println("SERVOS → " + color);
}

// ----------------------------
// MAIN LOOP
// ----------------------------
void loop() {

  // ----------------------------
  // BLUETOOTH CONTROL (NO DELAY)
  // ----------------------------
  if (BT.available()) {
    char cmd = BT.read();

    switch (cmd) {
      case 'F': motorForward(); break;
      case 'B': motorBackward(); break;
      case 'L': motorLeft(); break;
      case 'R': motorRight(); break;
      case 'S': motorStop(); break;
    }
  }

  // ----------------------------
  // COLOR READ (EVERY 120ms)
  // ----------------------------
  unsigned long now = millis();
  if (now - lastColorRead >= colorInterval) {
    lastColorRead = now;

    colorSensor.loop();

    int R = colorSensor.read_red();
    int G = colorSensor.read_green();
    int B = colorSensor.read_blue();

    String detected = "NONE";

    if (R > G + 20 && R > B + 20) detected = "RED";
    else if (G > R + 20 && G > B + 20) detected = "GREEN";
    else if (R > 80 && G > 80 && B > 80) detected = "WHITE";
    else if (abs(R - G) < 15 && abs(G - B) < 15) detected = "GRAY";

    setColorServos(detected);
  }
}
