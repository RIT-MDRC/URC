#include <Servo.h>

// Servo objects for each joint
Servo J1;
Servo J2;

// Command recieved from Serial
int cmd_J1 = 0;
int cmd_J2 = 0;

void setup() {
  // Designating pins that control each joint
  J1.attach(3);
  J2.attach(5);

  // Begin serial for debugging
  Serial.begin(9600);
  Serial.setTimeout(10000);

  // Home both axis
  J1.write(0);
  J2.write(0);
}

void loop() {
  while (Serial.available() > 0) {
    // Read first number
    int buf_J1 = Serial.parseInt();
    // Read first number
    int buf_J2 = Serial.parseInt();

    if (buf_J1 >= 0 && buf_J1 <= 180) {
      cmd_J1 = buf_J1;
      J1.write(cmd_J1);
    }
    if (buf_J2 >= 0 && buf_J2 <= 90) {
      cmd_J2 = buf_J2;
      J2.write(cmd_J2);
    }
    
    Serial.print("J1: ");
    Serial.print(cmd_J1);
    Serial.print(" - J2: ");
    Serial.print(cmd_J2);
    Serial.print("\n");
  }
}
