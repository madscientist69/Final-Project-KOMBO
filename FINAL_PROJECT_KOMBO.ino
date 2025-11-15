#include <tcs3200.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// ----------------------------
// PIN SETUP
// ----------------------------

// Bluetooth (SoftwareSerial)
SoftwareSerial BT(2, 3);  // RX, TX

// TCS3200 Sensor Pins
#define S0_PIN 4
#define S1_PIN 5
#define S2_PIN 6
#define S3_PIN 7
#define OUT_PIN 8

TCS3200 colorSensor(S0_PIN, S1_PIN, S2_PIN, S3_PIN, OUT_PIN);

// Servo Pins
Servo RServo;
Servo GServo;
Servo YServo;

// Motor driver L298N
#define IN1 12
#define IN2 13
#define IN3 A0
#define IN4 A1


// ----------------------------
// SETUP
// ----------------------------
void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  // Servo setup
  RServo.attach(9);
  GServo.attach(10);
  YServo.attach(11);

  // Motor setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Sensor setup
  colorSensor.begin();

  Serial.println("Calibrate WHITE...");
  colorSensor.calibrate_light();
  delay(1500);

  Serial.println("Calibrate BLACK...");
  colorSensor.calibrate_dark();
  delay(1500);

  colorSensor.calibrate();

  Serial.println("System Ready!");
}



// ----------------------------
// MOTOR CONTROL
// ----------------------------
void motorStop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

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



// ----------------------------
// MAIN LOOP
// ----------------------------
void loop() {

  // ----------------------------
  // 1. BACA DATA BLUETOOTH
  // ----------------------------
  if (BT.available()) {
    char cmd = BT.read();

    if (cmd == 'F') motorForward();
    else if (cmd == 'B') motorBackward();
    else if (cmd == 'L') motorLeft();
    else if (cmd == 'R') motorRight();
    else if (cmd == 'S') motorStop();
  }


  // ----------------------------
  // 2. BACA SENSOR WARNA
  // ----------------------------
  colorSensor.loop();

  int R = colorSensor.read_red();
  int G = colorSensor.read_green();
  int B = colorSensor.read_blue();

  Serial.print("R:"); Serial.print(R);
  Serial.print(" G:"); Serial.print(G);
  Serial.print(" B:"); Serial.println(B);

  int pos = 45;

  // ----------------------------
  // 3. DETEKSI WARNA
  // ----------------------------

  // RED
  if (R > G + 20 && R > B + 20) {
    RServo.write(pos + 45);
    GServo.write(pos);
    YServo.write(pos);
    Serial.println("Detected: RED");
  }

  // GREEN
  else if (G > R + 20 && G > B + 20) {
    GServo.write(pos + 45);
    RServo.write(pos);
    YServo.write(pos);
    Serial.println("Detected: GREEN");
  }

  // YELLOW (R & G tinggi, B rendah)
  else if (R > 80 && G > 80 && B < 40) {
    YServo.write(pos + 45);
    RServo.write(pos);
    GServo.write(pos);
    Serial.println("Detected: YELLOW");
  }

  // GRAY (semua warna hampir sama)
  else if (abs(R - G) < 15 && abs(G - B) < 15) {
    RServo.write(pos + 45);
    GServo.write(pos + 45);
    YServo.write(pos + 45);
    Serial.println("Detected: GRAY");
  }

  else {
    // No color
    RServo.write(pos);
    GServo.write(pos);
    YServo.write(pos);
    Serial.println("No color detected");
  }

  delay(100);
}
