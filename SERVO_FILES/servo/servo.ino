#include <ESP32Servo.h>

Servo myServo;  
int servoPin = 2;  // PWM-capable pin on ESP32

void setup() {
  Serial.begin(115200);  // Initialize serial monitor
  myServo.attach(servoPin, 500, 2500); // Attach servo with pulse width range
  Serial.println("Servo initialized.");
}

void loop() {
  Serial.println("Moving servo from 0° to 180°...");
  for (int pos = 0; pos <= 180; pos++) {
    myServo.write(pos);
    Serial.print("Servo Position: ");
    Serial.println(pos);
    delay(15);
  }

  Serial.println("Moving servo from 180° back to 0°...");
  for (int pos = 180; pos >= 0; pos--) {
    myServo.write(pos);
    Serial.print("Servo Position: ");
    Serial.println(pos);
    delay(15);
  }
}
