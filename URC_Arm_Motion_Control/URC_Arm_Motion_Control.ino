/*  6-Axis Robotic Arm Controller using I2C Drivers and reading commands from serial interface
 *  MICROCONTROLLER: Teensy 4.1 - https://www.pjrc.com/store/teensy41.html
 *  MOTOR/LINEAR CONTROLLER: Pololu SMC G2 18v25
 *  STEPPER CONTROLLER: Polulu Tic T249
 */
#include "Actuator.h"
#include <Servo.h>
#include <Encoder.h>

// VARIABLE/OBJECT DECLARATION ----------------------------------------------------------

//TIME VARIABLES
int timer;   // Holds time since last sending command to motor controller
int TIME;    // Holds last recorded time in milliseconds

//INPUT PINS
const int J3FeedbackPin = 41;
const int resetButtonPin = 23;
const int modeSwitchPin = 22;

bool automaticMode;

const uint8_t NUM_ACTUATORS = 7;

//I2C PINS - Just for reference
//const int SCL = 19;
//const int SDA = 18;

//ENCODER VARIABLES
Encoder enc_J1( 9,10);
Encoder enc_J2(11,12);

//ACTUATORS
// I2c Device, Actuator Type, Min Pos, Max Pos, Max Safe Speed, Acceleration, Units per Rev, Gear Ratio, Reverse Motion Direction
Actuator  Joint1(13, ActuatorType::MOTOR,    -90,  90,  600,  10,   1482.6, 2.5, false); // 75 : 30 reduction
Actuator  Joint2(14, ActuatorType::MOTOR,    -30,  80, 2000,  20, 1669.656,  30, false); // 30 : 1 reduction
Actuator  Joint3(15, ActuatorType::LINEAR,   200, 580, 3200, 200,      360,   1,  true);
Actuator  Joint4(16, ActuatorType::STEPPER,  -90,  90,  200,   1,   2*1036,   3,  true); // 1/2 Microstep and 3 : 1 reduction
Actuator   DiffA(17, ActuatorType::STEPPER, -720, 720,  200,   1,    8*200,   1, false); // 1/8th MicroSteps
Actuator   DiffB(18, ActuatorType::STEPPER, -720, 720,  200,   1,    8*200,   1, false); // 1/8th MicroSteps
Actuator Gripper(19, ActuatorType::MOTOR,      0,   0,    0,   0,        0,   1, false);
// NOTE: Units per rev is for OUTPUT SHAFT of ACTUATOR

//NOTE: Actuators DiffA/DiffB are combined into a differential gearbox. The combination of their motion determines the movement of joints 5 and 6
Actuator actuator[NUM_ACTUATORS] = {Joint1, Joint2, Joint3, Joint4, DiffA, DiffB, Gripper};

// SETUP -------------------------------------------------------------------------------

  // !!!!!!!! IMPORTANT !!!!!!!!!!
  //
  // Make sure motors are at home position (ZERO DEGREES) when turned on
  // If not, manually move arm to home position and reset
  //
  // !!!!!!!! IMPORTANT !!!!!!!!!!

void setup() {
  delay(1000);
  // Begin serial communication
  Serial.begin(115200);
  Serial.setTimeout(1);  // Stops reading from serial 1 millisecond after reading last character
  
  // Begin I2C communication
  Wire.begin();
  
  // Tell motor controller it is safe to move
  for (int n = 0; n < NUM_ACTUATORS; n++) {actuator[n].exitSafeStart();}

  // Set PID Algorithm gains for motor actuators
  actuator[0].setPIDGains(1,0,0);  // Base of Shoulder
  actuator[1].setPIDGains(1,0,0);  // Second axis of shoulder
  actuator[2].setPIDGains(15,0,0); // Elbow
  actuator[6].setPIDGains(1,0,0);  // Gripper

  // Initialize input for joint 3 linear actuator potentiometer pin
  pinMode(J3FeedbackPin,INPUT);  // Input pin for pot

  // Calibrate absolute encoder position of linear actuators (if actuator is not linear, this function will be ignored)
  float initialPos[NUM_ACTUATORS] = {0, 0, (float)analogRead(J3FeedbackPin), 0, 0, 0};
  for (int n = 0; n < NUM_ACTUATORS; n++) {
    actuator[n].calibratePos(initialPos[n]);
  }

  // Initialize input for reset button and debug/manual mode switch
  pinMode(resetButtonPin,INPUT);
  pinMode(modeSwitchPin,INPUT);

  // Initialize if automatic position control mode is active
  automaticMode = digitalRead(modeSwitchPin);  
  
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
  // Store current time in TIME variable to use next loop
  TIME = newTIME;
// READING COMMAND FROM SERIAL ***********************************************************
  /* COMMAND FORMAT: Cn v
   *    C - Command Identifier (char)
   *    n - Actuator Number (1,2,3,4,5,6)
   *    v - Command Value (int)
   * Valid Command Identifiers: 
   *    P - Desired Position
        S - Desired Speed
        R - Reset Actuator
        M - Maximum Safe Speed
        Q - Print to Serial
        K - PID Algorithm Gains
        H - Recalibrate home position
        E - Use end effector
   */
  // Check if command has been sent
  if (Serial.available()) {parseCommand();}
  
// ENCODER READING AND DECODING **********************************************************
  // If enough time has passed ...
  if (timer >= 20) {

    for (int n = 0; n < NUM_ACTUATORS; n++) {actuator[n].resetCommandTimeout();}

    // Read encoders
    float newPos[NUM_ACTUATORS] = {enc_J1.read(), enc_J2.read(), (float)analogRead(J3FeedbackPin), 0, 0, 0};
    // Pass encoder values to actuators
    for (int n = 0; n < NUM_ACTUATORS; n++) {actuator[n].interpretEncoder(newPos[n]);}

// MANUAL SPEED CONTROL VS AUTOMATIC POSITION CONTROL ********************************************************************
    // Detect if switch has been flipped and reset actuators
    switchMode();
    
    // If in automatic position mode, run position algorithm for all actuators (if stepper, this will be ignored)
    if (automaticMode) {for (int n = 0; n < NUM_ACTUATORS; n++) {actuator[n].positionAlgorithm(timer);}}
    
    // If reset button is pressed, reset every joint and set all state variables to zero
    if (digitalRead(resetButtonPin) == HIGH) {
      // Reset each actuator and store its current position
      for (int n = 0; n < NUM_ACTUATORS; n++) { actuator[n].reset(actuator[n].getCurrPos()); }
    }

    // Reset TIMER
    timer = 0;
  }
}

// FUNCTIONS ---------------------------------------------------------------------------------------------------------------------------

// Check if user is trying to switch between manual speed control and automatic position control
void switchMode() {
  // If switching to manual speed mode, clear all position algorithms
  if (digitalRead(modeSwitchPin) == HIGH && automaticMode) {
    // Turn automatic position control off
    automaticMode = false;

    // Reset actuators to clear position algorithm variables
    for (int n = 0; n < NUM_ACTUATORS; n++) {actuator[n].reset( actuator[n].getCurrPos() );}
    
    Serial.println(" MANUAL SPEED CONTROL ACTIVE");
  } 
  // If switching to automatic position mode, set all speeds to zero (without reseting anything else) to clear all manual speeds
  else if (digitalRead(modeSwitchPin) == LOW && !automaticMode) {
    // Turn automatic position control off
    automaticMode = true;

    // Clear any manually set speeds
    for (Actuator act : actuator) {act.sendTargetSpeed(0);}

    // Reset actuators to store current position
    for (int n = 0; n < NUM_ACTUATORS; n++) {actuator[n].reset( actuator[n].getCurrPos() );}
    
    Serial.println(" AUTOMATIC POSITION CONTROL ACTIVE");
  }
}
