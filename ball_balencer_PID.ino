#include <Servo.h>

// Servo Pins
#define SERVO_X_PIN 9
#define SERVO_Y_PIN 10
#define SERVO_Z_PIN 11

// Servo objects
Servo servoX;
Servo servoY;
Servo servoZ;

// PID variables
float Kp = 1.2;
float Ki = 0.01;
float Kd = 0.4;

float errorX = 0, errorY = 0;
float previousErrorX = 0, previousErrorY = 0;
float integralX = 0, integralY = 0;

// Target position from Raspberry Pi
float targetX = 90;
float targetY = 90;

// Update from Pi via serial
void readSerialTarget() {
  if (Serial.available() >= 2) {
    targetX = Serial.read();
    targetY = Serial.read();
  }
}

void setup() {
  Serial.begin(9600);
  servoX.attach(SERVO_X_PIN);
  servoY.attach(SERVO_Y_PIN);
  servoZ.attach(SERVO_Z_PIN); // optional - tilt axis or stabilizer
  servoX.write(90);
  servoY.write(90);
  servoZ.write(90);
}

void loop() {
  readSerialTarget();

  float currentX = analogRead(A0) / 10.23; // 0-100 scale (e.g., IMU or position sensor)
  float currentY = analogRead(A1) / 10.23;

  errorX = targetX - currentX;
  integralX += errorX;
  float derivativeX = errorX - previousErrorX;
  float outputX = Kp * errorX + Ki * integralX + Kd * derivativeX;
  previousErrorX = errorX;

  errorY = targetY - currentY;
  integralY += errorY;
  float derivativeY = errorY - previousErrorY;
  float outputY = Kp * errorY + Ki * integralY + Kd * derivativeY;
  previousErrorY = errorY;

  servoX.write(constrain(90 + outputX, 0, 180));
  servoY.write(constrain(90 + outputY, 0, 180));
  servoZ.write(90);  // static or used for extra stabilization
  delay(50);
}
