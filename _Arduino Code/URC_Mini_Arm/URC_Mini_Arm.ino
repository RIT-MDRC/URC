/*  Controller for URC mini-arm
 *  MICROCONTROLLER: Teensy 4.1 - https://www.pjrc.com/store/teensy41.html
 *  BLUETOOTH MODULE: HC-05
 */
#include "Joint.h"

Joint ShouldA('1', 14,  14, 1023, -110, 180, false);
Joint ShouldB('2', 15, 540, 1023,  -20, 120,  true);
Joint   Elbow('3', 16, 523, 1010,   20, 165,  true);
Joint  WristA('4', 17,   0,  620,  -90,  80,  true);
// ONLY FIRST 4 ARE ACTUALLY BUILT. REMAINING JOINTS ARE PLACE HOLDERS
Joint  WristB('5', 18,   0, 1023,    0,   0, false);
Joint  WristC('6', 19,   0, 1023,    0,   0, false);

// Put joints into array for easy access
Joint joints[6] = {ShouldA, ShouldB, Elbow, WristA, WristB, WristC};
int NUM_JOINTS = 6;

// Analog input pin for gripper control switch
int gripper_pin = 20;
// Direction gripper is moving (1 = CLOSING, 0 = HOLDING, -1 = OPENING)
int gripper_dir = 0;
// Last loop's direction for detecting a change
int last_dir = 0;

// Analog pins used to read potentiometers
//int potPin[7] = {14, 15, 16, 17, 18, 19, 20};

// Pin for SEND DATA button (pressed to send joint positions to big arm)
int button_pin = 32;

// Was button pressed on last loop?
bool lastButtonState = false;

//TIME VARIABLES
int timer;   // Holds time since last sending command to motor controller
int TIME;         // Holds last recorded time in milliseconds
int t = 0;
 
void setup() {
  // Serial Terminal
  Serial.begin(115200);

  // UART with Bluetooth Module
  Serial2.begin(38400);
  
  // Initialize analog input of joints
  for (int n = 0; n < NUM_JOINTS; n++) {joints[n].init();}

  // Initialize gripper control switch input
  pinMode(gripper_pin, INPUT);

  // Initialize activation switch and read its state
  pinMode(button_pin,INPUT);
  lastButtonState = digitalRead(button_pin);

  // Initialize time variables
  timer = 0;
  TIME = millis();
}

void loop() {
  // Read the current time 
  int newTIME = millis();
  // Calculate change in time since last reading and keep track with timer
  timer += newTIME - TIME;
  // Store current time in TIME variable to use next loop
  TIME = newTIME;

  if (timer > 50) {
    // Read from all the potentiometers
    for (int n = 0; n < NUM_JOINTS; n++) {joints[n].read();}

    // Read from gripper control switch and send command if a state change is detected
    readGripperSwitch();

    // Read SEND DATA button and detect rising edge
    bool buttonState = digitalRead(button_pin);
    if (buttonState == HIGH && lastButtonState == LOW) {
      // When button pressed, send gcode for each joint
      for (int n = 0; n < NUM_JOINTS; n++) {Serial2.println(joints[n].gcode());}
    }
    // Store current button state for next loop
    lastButtonState = buttonState;

    // Reset timer
    timer = 0;
  }
}
