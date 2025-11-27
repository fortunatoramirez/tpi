#include <Servo.h>

Servo plate;
const int SERVO_PIN = 9;

void setup() {
  Serial.begin(9600);
  plate.attach(SERVO_PIN);
  plate.writeMicroseconds(1500); // parado
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();

    if (c == 'L') {            // Left
      plate.writeMicroseconds(1400);
    } 
    else if (c == 'R') {       // Right
      plate.writeMicroseconds(1600);
    } 
    else if (c == 'S') {       // Stop
      plate.writeMicroseconds(1500);
    }
  }
}
