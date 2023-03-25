#include <Tic.h>

// Tic Stepper Controller Objects
TicI2C diffA(16);
TicI2C diffB(17);

// Use to reverse motion of steppers in code instead of changing wiring
bool reverseA = false;
bool reserseB = false;

// MOTION CONSTANTS
float YAW_MAX = 90;   // Degrees
float YAW_MIN = -90;  // Degrees
float SPEED = 100;    // Steps per second
float STEPS_PER_REV = 200 * 8; // (200 Full Steps/Rev) / (1/8 Steps)

float yaw = 0;
float roll = 0;

void setup()
{
  // Set up I2C.
  Wire.begin();

  // Set up Serial Terminal
  Serial.begin(115200);
  Serial.setTimeout(0.5);
  
  // Give the Tic some time to start up.
  delay(20);

  // Set the Tic's current position to 0, so that when we command
  // it to move later, it will move a predictable amount.
  diffA.haltAndSetPosition(0);
  diffB.haltAndSetPosition(0);

  // Tells the Tic that it is OK to start driving the motor.
  diffA.exitSafeStart();
  diffB.exitSafeStart();
}

//TIME VARIABLES
int timer;   // Holds time since last sending command to motor controller
int TIME;         // Holds last recorded time in milliseconds
int t = 0;

void loop() {
  // Read the current time 
  int newTIME = millis();
  // Calculate change in time since last reading and keep track with timer
  timer += newTIME - TIME;
  t += newTIME - TIME;
  // Store current time in TIME variable to use next loop
  TIME = newTIME;

  
  // READ FROM SERIAL TO GET DESIRED POSITION IN DEGREES
  if (Serial.available()) {
    yaw = Serial.parseFloat();
    roll = Serial.parseFloat(); 

    // CHECK IS DESIRED POSITION IS WITHIN BOUNDS
    if (yaw > YAW_MAX) {yaw = 90;}
    else if (yaw < YAW_MIN) {yaw = -90;}

    Serial.print("YAW: ");
    Serial.print(yaw);
    Serial.print(", ROLL:");
    Serial.println(roll);

    // Convert desired positions from degrees to steps
    float yaw_steps = yaw / 360 * STEPS_PER_REV;
    float roll_steps = roll / 360 * STEPS_PER_REV;

    // Calculate needed position of each motor to achieve desired rotations
    // Uses yaw = (thetaA + thetaB)/2 and roll = (thetaA - thetaB)/2
    float thetaA = roll_steps - yaw_steps;
    float thetaB = roll_steps + yaw_steps;
    
    // Send desired positions to steppers
    diffA.setTargetPosition(thetaA);
    diffB.setTargetPosition(thetaB);
  }
  
  // If 50ms have passed, send hearbeat pulse to reset command timeout
  if (timer > 50) {
    diffA.resetCommandTimeout();
    diffB.resetCommandTimeout();
    // Reset timer
    timer = 0;
  }
    
}
