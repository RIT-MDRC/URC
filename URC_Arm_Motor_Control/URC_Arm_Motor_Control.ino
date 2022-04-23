/*  Position control of a motor with a quadrature encoder
 *  MICROCONTROLLER: Teensy 4.1
 *  MOTOR CONTROLLER: Pololu SMC G2 18v25
 */
#include "Motor.h"

// VARIABLE/OBJECT DECLARATION ----------------------------------------------------------

//TIME VARIABLES
int timer;   // Holds time since last sending command to motor controller
int TIME;         // Holds last recorded time in milliseconds
int t = 0;

//POTENTIOMETER VARIABLES
const int potPin = 41;

const uint8_t NUM_JOINTS = 3;

//ENCODER VARIABLES
Encoder enc_J1(16,17);
Encoder enc_J2(14,15);

//MOTORS
// minPos, maxPos, pulses_per_rev (of output shaft), gear ratio, i2c device number, default max speed, acceleration max, movement threshold
Motor J1(-90, 90, 1482.6, 2.5, 13, 500, 50, 30);  // RATIO = 75 : 30
Motor J2(-10, 80, 1669.656, 30, 14, 2000, 100, 30); //RATIO = 15 : 1
Motor J3(670, 860, 360, 1, 15, 3200, 200, 20);

Motor joints[NUM_JOINTS] = {J1, J2, J3};

// FUNCTIONS ---------------------------------------------------------------------------

// SETUP -------------------------------------------------------------------------------

  // !!!!!!!! IMPORTANT !!!!!!!!!!
  //
  // Make sure motors are at home position (ZERO DEGREES) when turned on
  // If not, manually move arm to home position and reset
  //
  // !!!!!!!! IMPORTANT !!!!!!!!!!

void setup() {
  // Begin I2C communication
  Wire.begin();
  // Tell motor controller it is safe to move
  
  for (Motor m : joints) {m.exitSafeStart();}

  delay(1000);
  // Begin serial communication
  Serial.begin(115200);
  Serial.setTimeout(10);

  // Initialize linear actuator potentiometer feedback
  pinMode(potPin,INPUT);  // Input pin for pot
  joints[2].setPosition(analogRead(potPin));

  // Initialize time variables
  timer = 0;
  TIME = millis();
}

// MAIN LOOP ---------------------------------------------------------------------------

void loop() {
  // Read the current time 
  int newTIME = millis();
  // Calculate change in time since last reading and keep track with timer
  timer += newTIME - TIME;
  t += newTIME - TIME;
  // Store current time in TIME variable to use next loop
  TIME = newTIME;
// READING COMMAND FROM SERIAL ***********************************************************
  /* Commands expected as:
   *    DESIGNATOR_CHAR VALUE
   * Valid DESIGNATOR CHARACTERS are P - Postion Set, S - Max Speed, R - Reset Drivers, Q - Print to Serial, M - Move, H - Homing
   */
  if (Serial.available() > 1) {

    float setSpeed = 0;
    float setPos = 0; 

    // Read command identifier
    char cmd = (char) Serial.read();
    int n = Serial.parseInt() - 1;
    
    switch (cmd) {
      case 'P':
        // Set new command position
        setPos = Serial.parseFloat();
        joints[n].setPosition(setPos);
        
        Serial.print("POSITION: ");
        Serial.println(setPos);
        
        break;
      case 'S':
        // Set new max speed
        setSpeed = Serial.parseFloat();
        if (joints[n].setMaxSpeed(setSpeed)) {        
          /*
          Serial.print("SPEED: ");
          Serial.println(setSpeed);
          */
        }
        break;
      case 'R':
        // Reset motor driver and set position and speed to zero
        joints[n].reset();

        switch (n) {
          case 0:
            enc_J1.write(0);
            break;
          case 1:
            enc_J2.write(0);
            break;
          case 2:
            joints[n].setPosition(analogRead(potPin));
            break;
        }
        Serial.print("RESET ");
        Serial.println(n+1);
        
        break;
      case 'Q':
        // Print status of motor
        Serial.print("JOINT ");
        Serial.print(n+1);
        Serial.print(": ");
        joints[n].print();
        break;
      case 'M':
        //Move Motor
        setSpeed = Serial.parseFloat();
        joints[n].moveMotor(setSpeed);
        break;
      default:
        break;
    }
  }
  
// ENCODER READING AND DECODING **********************************************************
  // If enough time has passed ...
  if (timer >= 20) {

    float newPos[NUM_JOINTS] = {enc_J1.read(), enc_J2.read(), (float)analogRead(potPin)};
    
    // Read and interpret the encoder
    for (int n = 0; n < NUM_JOINTS; n++) {
      joints[n].interpretEncoder(newPos[n]);
    }

    /* POSITION ALGORITHM - Doesn't work well
    // Check if the Emergency Stop Button has been pressed
    if (digitalRead(23) == 1) {
      //Serial.println("ENABLED");
      for (int n = 0; n < NUM_JOINTS; n++) {
        float newSpeed = joints[n].applyAccel();
        joints[n].sendMotorSpeed((int16_t)newSpeed);
      }
    } else {
      for (int n = 0; n < NUM_JOINTS; n++) {joints[n].sendMotorSpeed(0);}
      //Serial.println("DISABLED");
    }
    */
    
    // Reset TIMER
    timer = 0;
  }
/*
  if (t < 3000) {
    joints[0].setPosition(500);
    joints[1].setPosition(8000);
  } else if (t > 3000 && t < 6000) {
    joints[0].setPosition(-500);
    joints[1].setPosition(2000);
  } else if (t > 6000) {t = 0;}
  */
}
