#include "Motor.h"

Motor::Motor(float minPos_degrees, float maxPos_degrees, double pulses, double ratio, uint8_t deviceNum, float defaultSpeed, float defaultAccel) {
    // MOTOR CONSTANTS
     this->PULSE_PER_REV = pulses;
     this->GEAR_RATIO = ratio;
     this->I2C_NUM = deviceNum;   // I2C Device # of motor controller

    // Convert limits from degrees to encoder pulses
     this->MIN_POS = degToPulse(minPos_degrees);
     this->MAX_POS = degToPulse(maxPos_degrees);

    // Initialize motion contrainsts to defaults
     this->cmdSpeed = defaultSpeed;
     this->accel = defaultAccel;     
    
    // Initialize state and position algorithm variables to zero
     this->currPos = 0;
     this->cmdPos = 0;
     this->currSpeed = 0;
     this->encoderSpeed = 0;
     this->encoderAccel = 0;
     this->initialPos = 0;
     this->dir_travel = FORWARD;
     this->dir_initialSpeed = FORWARD;

     // Initialize flag variables to false
     this->error = false;
     this->overshooting = false;
     this->started_opposite = false;
}

// Required to allow motors to move.
// Must be called when controller restarts and after any error.
void Motor::exitSafeStart() {
  Wire.beginTransmission(this->I2C_NUM);
  Wire.write(0x83);  // Exit safe start
  Wire.endTransmission();

  // Output message to serial port to confirm command sent
  Serial.print("I2C Driver ");
  Serial.print(this->I2C_NUM);
  Serial.println(" exited safe start");
}

// Set motor speed and direction
// INPUT: Directional motor speed [between 0 and 3200]
void Motor::sendMotorSpeed(int16_t speed) {
  uint8_t cmd = 0x85; // Hexidecimnal code for sending FORWARD speed

  // If it is unsafe to move motor, speed is set to zero
  if (!this->error) {
    // If speed is negative, get absolute value and change I2C command to REVERSE
    if (speed < 0)  {
      cmd = 0x86;  // Hexidecimnal code for sending REVERSE speed
      speed = (int16_t) abs(speed);
    }
  } else {speed = 0;}
  
  // Send the command (speed must be broken into 2 pcs to fit inside write function)
  Wire.beginTransmission(this->I2C_NUM);
  Wire.write(cmd);
  Wire.write(speed & 0x1F);
  Wire.write(speed >> 5 & 0x7F);
  Wire.endTransmission();
}

// Ask the driver how long motor has been active (time since exiting safe start)
uint16_t Motor::readUpTime() {
  Wire.beginTransmission(this->I2C_NUM);
  Wire.write(0xA1);  // Command: Get variable
  Wire.write(28);    // Variable ID: Up time (low)
  Wire.endTransmission();
  Wire.requestFrom(this->I2C_NUM, (uint8_t)2);
  uint16_t upTime = Wire.read();
  upTime |= Wire.read() << 8;
  return upTime;
}

// Sets the new desired position of the motor after checking if within limits, then calculate path to reach it
// INPUT: Desired position [in degrees]
void Motor::setNewPosition(float newPos_degrees) {
  // Convert desired position from degrees to encoder pulses
  float newPos = degToPulse(newPos_degrees);

  // Check if desired position is within safe limits
  if (newPos <= this->MAX_POS && newPos >= this->MIN_POS) {
    // If desired position is in the safe range, set command position to desired position
    this->cmdPos = newPos;
  } else if (newPos > this->MAX_POS) {
    // If desired pos is above safe max, set command position to max position
    this->cmdPos = this->MAX_POS;
  } else if (newPos < this->MIN_POS) {
    // If desired pos is below safe min, set command position to min position
    this->cmdPos = this->MIN_POS;
  } else {
    // If you're here, something has gone terribly wrong
    Serial.println("Why are you here?");
    return;
  }

  // PRE-CALCULATE PATH TO DESIRED POSITION ---------------------------

  // Capture the current position of the motor;
  this->initialPos = this->currPos;

  // Determine direction from initial to final pos (defaults to initial vel direction when final pos is same as initial pos)
  this->dir_travel = this->dir_initialSpeed;
  if (this->cmdPos != this->initialPos) {
    this->dir_travel = (this->cmdPos - this->initialPos) / abs(this->cmdPos - this->initialPos);
  }
  
  // Change in position required for motor to accelerate to and decelerate from max velocity
  float delta_pos_MaxVel = ( 2*pow(this->cmdSpeed,2) - pow(this->encoderSpeed,2) ) / (2*this->accel);

  // Change in position required to deccelerate from max velocity (with correction factor)
  float delta_pos_MaxDeccel = pow(this->cmdSpeed,2) / (2*this->accel);

  // Total distance from initial and final positions
  float delta_pos_Total = abs(this->cmdPos - this->initialPos);

  // Total distance required to stop if max speed not limited
  float delta_pos_Deccel = (delta_pos_Total + pow(this->currSpeed,2)/(2*this->accel) ) / 2;

  // Total distance required to stop from initial speed
  float delta_pos_InstantDeccel = pow(this->currSpeed,2) / (2*this->accel);

  // Default to max speed and no overshot
  this->delta_pos_SafeStop = delta_pos_MaxDeccel;
  this->overshooting = false;

  // Is motor already in final position and sitting still?
  if (this->currSpeed == 0 && this->cmdPos == this->initialPos) {
    // Initial pos = final pos and initial velocity = 0
    this->delta_pos_SafeStop = 0;
    Serial.println(" !!! EDGE CASE: Motor is already in position and sitting still !!!");
  
  // Will motor overshoot position because the initial velocity is too high?
  // Only edge case if initial velocity is in travel direction
  } else if (delta_pos_Total < delta_pos_InstantDeccel) {
    // Stopping distance will be half of overshot distance
    this->delta_pos_SafeStop = (delta_pos_InstantDeccel - delta_pos_Total) / 2;
    // Invert direction so motor immediately deccelerates
    // If initial vel is opposite to travel direction, direction will be inverted once motor has returned to initial position (i.e. inside loop)
    this->started_opposite = true;
    if (this->dir_initialSpeed == this->dir_travel) {
        this->dir_travel = -1 * this->dir_travel;
        this->started_opposite = false;
    }
    // Set OVERSHOOTING flag so stopping distance is only detected coming back
    this->overshooting = true;
    Serial.println(" !!! EDGE CASE : Initial velocity is too high, Calculating correction for overshot !!!");
  
  // Does motor has enough distance to travel to speed up to and down from maximum speed
  } else if (delta_pos_Total < delta_pos_MaxVel) {
    // Stopping distance from max possible speed will be used
    this->delta_pos_SafeStop = delta_pos_Deccel;
    Serial.println("!!! EDGE CASE : Cannot reach max speed, Calculating new speed limit !!!");
  }
}

// Set the new maximum allowable speed for the motor
// INPUT: desired speed [between 0 and 3200]
// OUTPUT: Can the desired speed be allowed? [True or False]
bool Motor::setMaxSpeed(float newSpeed) {
  if (abs(newSpeed) <= this->ABS_MAX_SPEED) {
     this->cmdSpeed = abs(newSpeed);
    return true;
  }
  return false;
}

// Determine velocity and accelerations [in pulses per code loop] based on encoder readings
// INPUT: new position reading of the encoder
void Motor::interpretEncoder(float newPos) {
  // Read the new postion of the motor encoder and calculate change since last cycle
  float last_encoderSpeed = this->encoderSpeed;
   this->encoderSpeed = newPos - this->currPos;
   this->encoderAccel = this->encoderSpeed - last_encoderSpeed;
  // Update postion variable
   this->currPos = newPos;

  /*
  // Check if change in postion is opposite of desired direction
  if ((encoderSpeed < 0 && dir_speed == FORWARD) || (encoderSpeed > 0 && dir_speed == REVERSE)) {
    // If yes, alert user
    Serial.println("REVERSE");
  }
  */
}

// User drives motor by manually specifying speed
// Will automatically limit speed when it approaches max/min positions
// INPUT: percent of max motor speed [-100 to 100]
void Motor::driveMotor(float newPercent) {
  float deltaPos = 0;
  float nPos = 0;
  float percent_speed = 0;
  float moveThreshold = this->getMaxPos() * 0.1;

  // Check if motor is within bounds and raise warning if
  if (this->getCurrPos() > this->getMaxPos() || this->getCurrPos() < this->getMinPos()) {this->stop(1);}

  //Calculate the difference between the current position and the endstop threshold which the arm would be moving towards
  if(newPercent > 0){
    deltaPos = this->getMaxPos() - moveThreshold - this->getCurrPos(); //neg if within threshold, otherwise positive distance
  } else if (newPercent < 0) {
    deltaPos = this->getCurrPos() - (this->getMinPos() + moveThreshold); //neg if within threshold, otherwise positive distance
  } else {
    deltaPos = -1 * moveThreshold;  // 0 Speed
  }

  //If negative, can go full speed, otherwise calculate a cosine interpolation speed
  if(deltaPos > 0){
    nPos = PI/2;
  } else {
    nPos = map(abs(deltaPos), 0, moveThreshold, PI/2, 0);
  }
  
  float maxPercent = (-cos(nPos) + 1) * 100 * abs(newPercent)/newPercent;

  //If the requested speed is acceptable, use it, otherwise use the calculated max speed
  if(abs(newPercent) > abs(maxPercent)){
    percent_speed = maxPercent;
  } else {
    percent_speed = newPercent;
  }

  Serial.print(percent_speed);
  Serial.print(" , ");
  Serial.println(maxPercent);
  
  //Convert percent to actual speed and send 
  if (!this->error) {
    this->currSpeed = percent_speed / 100 * this->cmdSpeed;
    this->sendMotorSpeed(currSpeed);
  }
}

// Use path calculated by setNewPosition to move joint to specified position
void Motor::positionAlgorithm() {
  float newSpeed = 0;

  // Only run algorithm if not in an error state;
  if (!this->error) {
    // Is is close enough to start slowing down?
    if (abs(this->cmdPos - this->currPos) > this->delta_pos_SafeStop) {
      // Apply acceleration
      newSpeed = this->currSpeed + this->accel*this->dir_travel;
      // Cap speed at MAX speed
      if (abs(newSpeed) > this->cmdSpeed) {newSpeed = this->cmdSpeed*this->dir_travel;}
    
      // If in the overshot edge case but initial velocity is opposite to travel direction, direction needs to be flipped once motor 
      // returns to initial position so it begins deccelerating
      if (this->overshooting && this->started_opposite && dir_initialSpeed == -this->dir_travel && (this->currPos - this->initialPos)*this->dir_travel > 0) {
        this->dir_travel = -this->dir_travel;
      }
    // Is this the overshot edge case? Are we overshooting? Are we still going the wrong way?
    } else if (this->overshooting && abs(this->cmdPos - this->currPos) <= this->delta_pos_SafeStop && this->currSpeed/abs(this->currSpeed) != this->dir_travel) {
      newSpeed = this->currSpeed + this->accel * this->dir_travel;

    // Slowing down
    } else {
      // Apply acceleration in reverse
      newSpeed = this->currSpeed - this->accel*this->dir_travel;
      // Keep motor from overshooting and reversing direction
      if (newSpeed*this->dir_travel < 0) {newSpeed = 0;}
      this->overshooting = false; // Otherwise, motor will keep trying to correct and begin oscilating
    }

    this->sendMotorSpeed(newSpeed);
  }
}

//Home motor joint by moving at a slow speed.
void Motor::homing(int16_t dir)  {
  if(dir > 0){
    sendMotorSpeed(0.75*this->cmdSpeed);
    this->currSpeed = 0.75*this->cmdSpeed;
  }
  else if(dir < 0){
    sendMotorSpeed(-0.75*this->cmdSpeed);
    this->currSpeed = -0.75*this->cmdSpeed;
  }
  else{
    sendMotorSpeed(0);
  }
}

// Stops motor, clears error state, resets I2C communication line, clears all state variables, and set new current position
// INPUT: New current position
void Motor::reset(float pos) {
  // Stop motor
  this->sendMotorSpeed(0);
  
  // Set new current position after checking if new position is within bounds
  if (pos > this->MAX_POS) {this->currPos = this->MAX_POS;}
  else if (pos < this->MIN_POS) {this->currPos = this->MIN_POS;}
  else {this->currPos = pos;}
  
  // Clear all state variables
  this->cmdPos = 0;
  this->currSpeed = 0;

  // Reset I2C
  this->exitSafeStart();

  // Clear error state
  this->error = false;
}
void Motor::reset() {this->reset(0);}

// Stops motor and displays error message to serial terminal
// INPUT: error code
void Motor::stop(int errorCode) {
  this->error = true;
  this->currSpeed = 0;
  this->sendMotorSpeed(0);

  Serial.print(" !!! WARNING: Motor has been stopped because ");
  switch (errorCode) {
    case 0:
      Serial.println("user told it to stop ");
      break;
    case 1:
      Serial.println("MIN or MAX position was exceeded !!!");
      break;
    default:
      Serial.println("of unknown error !!!");
      break;
  }
}

void Motor::print() {
  Serial.print(this->currPos);
  Serial.print(",");
  Serial.print(this->cmdPos);
  Serial.print(",");
  Serial.print(this->currSpeed);
  Serial.print(",");
  Serial.println(this->cmdSpeed);
}

//Getter funcs for private vars
float Motor::getCurrPos() {return currPos;}
float Motor::getCmdPos() {return cmdPos;}
float Motor::getMinPos() {return MIN_POS;}
float Motor::getMaxPos() {return MAX_POS;}

//Calculate encoder Count from degrees
float Motor::degToPulse(float deg)  {
  return deg / 360 * this->PULSE_PER_REV * this->GEAR_RATIO;
}
