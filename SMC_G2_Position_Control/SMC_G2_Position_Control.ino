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

const uint8_t NUM_JOINTS = 2;

//ENCODER VARIABLES
Encoder enc_J1(16,17);
Encoder enc_J2(14,15);

//MOTORS
Motor J1(1482.6, 52.95, 13, 500, 50);
Motor J2(6678.624, 139.138, 14, 1500, 150);

Motor joints[NUM_JOINTS] = {J1, J2};

// FUNCTIONS ---------------------------------------------------------------------------

// SETUP -------------------------------------------------------------------------------

void setup() {
  // Begin I2C communication
  Wire.begin();
  // Tell motor controller it is safe to move
  
  for (Motor m : joints) {m.exitSafeStart();}

  delay(1000);
  // Begin serial communication
  Serial.begin(115200);
  Serial.setTimeout(10);

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
   * Valid DESIGNATOR CHARACTERS are P - Postion, S - Max Speed
   */
  if (Serial.available() > 1) {

    float setSpeed = 0;
    float setPos = 0; 
    char cmd = (char) Serial.read();
    int n = 0;
    
    switch (cmd) {
      case 'P':
        n = Serial.parseInt() - 1;
        // Set new command position
        setPos = Serial.parseFloat();
        joints[n].setPosition(setPos);
        
        Serial.print("POSITION: ");
        Serial.println(setPos);
        
        break;
      case 'S':
        // Set new max speed
        n = Serial.parseInt() - 1;
        setSpeed = Serial.parseFloat();
        if (joints[n].setMaxSpeed(setSpeed)) {        
          /*
          Serial.print("SPEED: ");
          Serial.println(setSpeed);
          */
        }
        break;
      case 'R':
        n = Serial.parseInt() - 1;
        joints[n].exitSafeStart();
        break;
      case 'Q':
        n = Serial.parseInt() - 1;
        // Print motor status
        setPos = Serial.parseFloat();
        joints[n].print();
        break;
      default:
        break;
    }
  }

// ENCODER READING AND DECODING **********************************************************
  // If enough time has passed ...
  if (timer >= 20) {

    float newPos[NUM_JOINTS] = {enc_J1.read(), enc_J2.read()};
    
    // Read and interpret the encoder
    for (int n = 0; n < NUM_JOINTS; n++) {
      joints[n].interpretEncoder(newPos[n]);
    }
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
