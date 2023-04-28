#include "Joint.h"

// CONSTRUCTOR
Joint::Joint(char joint_num, int pot_pin, int max_pot_val, int min_pot_val, int max_joint_pos, int min_joint_pos, bool reverse) {
  // Set constants
  this->JOINT_NUM = joint_num;
  this->POT_PIN = pot_pin;
  this->MAX_POT_VALUE = max_pot_val;
  this->MIN_POT_VALUE = min_pot_val;
  this->MAX_JOINT_POS = max_joint_pos;
  this->MIN_JOINT_POS = min_joint_pos;
  this->REVERSED = reverse;

  // Initialize state variables to zero
  setPotValue(0);
  setJointPos(0);
}

// Initialize analog input pin for potentiometer, then read initial value
void Joint::init() {
  pinMode(getPotPin(),INPUT);
  read();
}

// Read from potentiometer, then convert reading to joint position in degrees
void Joint::read() {
  // Read potentiometer then store value
  int val = analogRead(getPotPin());
  setPotValue(val);

  // Convert potentiometer reading to joint position in degrees then store
  int pos = 0;
  if (isReversed()) {pos = map(val, getMinPotValue(), getMaxPotValue(), getMaxJointPos(), getMinJointPos());}
  else {pos = map(val, getMinPotValue(), getMaxPotValue(), getMinJointPos(), getMaxJointPos());}
  
  setJointPos(pos);
}

// Print joint position to Serial Terminal 
void Joint::print() {
  Serial.print(getJointNum());
  Serial.print(": ");
  Serial.print(getJointPos());
  Serial.print(", ");
  // NOTE: Make sure to print a newline character to println after calling this function
}

// Create G-CODE for sending to big arm
String Joint::gcode() {
  // Initialize command format (second char will always be 'P' for setting position)
  char gcode[7] = "0P0000";
  // Set first character to be joint number 
  gcode[0] = getJointNum();

  // Get joint position
  int pos = getJointPos();

  // Check if negative sign is needed
  if (pos < 0) {
    // Add negative sign right after command identifier
    gcode[2] = '-';
    // Make position value position for rest of calculations
    pos *= -1;
  }

  // Split position value into digits then convert to char (ASCII value of numbers are 48-57)
  gcode[3] = (char) (pos / 100 + 48);
  gcode[4] = (char) (pos / 10 % 10 + 48);
  gcode[5] = (char) (pos % 10 + 48);

  return String(gcode);
}

//GETTERS
// State Variables
int Joint::getPotValue() {return this->pot_value;}
int Joint::getJointPos() {return this->joint_pos;}
// Constants
int Joint::getMinPotValue() {return this->MIN_POT_VALUE;}
int Joint::getMaxPotValue() {return this->MAX_POT_VALUE;}
int Joint::getMinJointPos() {return this->MIN_JOINT_POS;}
int Joint::getMaxJointPos() {return this->MAX_JOINT_POS;}
int Joint::getPotPin() {return this->POT_PIN;}
char Joint::getJointNum() {return this->JOINT_NUM;}
bool Joint::isReversed() {return this->REVERSED;}

//SETTERS
void Joint::setPotValue(int val) {this->pot_value = val;}
void Joint::setJointPos(int pos) {this->joint_pos = pos;}
    
