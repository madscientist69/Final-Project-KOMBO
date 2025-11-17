#include <tcs3200.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// ----------------------------
// BLUETOOTH
// ----------------------------
SoftwareSerial BT(2, 3);  // RX, TX

// ----------------------------
// TCS3200 COLOR SENSOR
// ----------------------------
#define S0_PIN 4
#define S1_PIN 5
#define S2_PIN 6
#define S3_PIN 7
#define OUT_PIN 8
TCS3200 colorSensor(S0_PIN, S1_PIN, S2_PIN, S3_PIN, OUT_PIN);

// ----------------------------
// SERVOS on ANALOG PINS
// ----------------------------
Servo RServo;   // RED
Servo GServo;   // GREEN
Servo WServo;   // WHITE

// A2, A3, A4 are usable as digital pins
#define RSERVO_PIN A2
#define GSERVO_PIN A3
#define WSERVO_PIN A4

// ----------------------------
// L293D MOTOR DRIVER
// ----------------------------
const int motorA1     = 3;
const int motorA2     = 4;
const int motorAspeed = 5;

const int motorB1     = 7;
const int motorB2     = 8;
const int motorBspeed = 6;

int state;
int vSpeed = 200;    // 0–255

// ----------------------------
// TIMING
// ----------------------------
unsigned long lastColorRead = 0;
unsigned long colorInterval = 120;
String lastColor = "NONE";

void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  // Servo setup
  RServo.attach(RSERVO_PIN);
  GServo.attach(GSERVO_PIN);
  WServo.attach(WSERVO_PIN);

  // Motor pins
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  // Sensor calibration
  colorSensor.begin();
  colorSensor.calibrate_light();
  delay(1000);
  colorSensor.calibrate_dark();
  delay(1000);
  colorSensor.calibrate();

  Serial.println("READY");
}

// -------------------------------------------------------
// SERVO FUNCTION (no jitter, only move if color changes)
// -------------------------------------------------------
void setColorServos(String color) {
  if (color == lastColor) return;
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
// MOVE CAR FUNCTIONS (L293D)
// ----------------------------
void motorForward() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorB2, HIGH);
  analogWrite(motorAspeed, vSpeed);
  analogWrite(motorBspeed, vSpeed);
}

void motorBackward() {
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB1, HIGH);
  digitalWrite(motorB2, LOW);
  analogWrite(motorAspeed, vSpeed);
  analogWrite(motorBspeed, vSpeed);
}

void motorLeft() {
  digitalWrite(motorA2, HIGH);
  digitalWrite(motorA1, LOW);
  digitalWrite(motorB2, LOW);
  digitalWrite(motorB1, HIGH);
  analogWrite(motorAspeed, vSpeed);
  analogWrite(motorBspeed, vSpeed);
}

void motorRight() {
  digitalWrite(motorA2, LOW);
  digitalWrite(motorA1, HIGH);
  digitalWrite(motorB2, HIGH);
  digitalWrite(motorB1, LOW);
  analogWrite(motorAspeed, vSpeed);
  analogWrite(motorBspeed, vSpeed);
}

void motorStop() {
  analogWrite(motorAspeed, 0);
  analogWrite(motorBspeed, 0);
}

// ----------------------------
// MAIN LOOP
// ----------------------------
void loop() {

  // ----------------------------
  // BLUETOOTH COMMANDS
  // ----------------------------
  if (BT.available()) {
    state = BT.read();
    Serial.println(state);

    // SPEED CONTROL 0–4
    if (state == '0') vSpeed = 0;
    else if (state == '1') vSpeed = 100;
    else if (state == '2') vSpeed = 180;
    else if (state == '3') vSpeed = 200;
    else if (state == '4') vSpeed = 255;

    // DIRECTION
    if (state == 'F') motorForward();
    else if (state == 'B') motorBackward();
    else if (state == 'L') motorLeft();
    else if (state == 'R') motorRight();
    else if (state == 'S') motorStop();
  }

  // ----------------------------
  // READ COLOR SENSOR EVERY 120ms
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
