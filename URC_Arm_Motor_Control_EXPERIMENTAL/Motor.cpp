#include "Motor.h"

Motor::Motor(float minPos_degrees, float maxPos_degrees, double pulses, double ratio, uint8_t deviceNum, float defaultSpeed, float defaultAccel, float moveThreshold) {
    // MOTOR CONSTANTS
     this->PULSE_PER_REV = pulses;
     this->GEAR_RATIO = ratio;
     this->I2C_NUM = deviceNum;   // I2C Device # of motor controller
     this->MOVE_THRESHOLD = moveThreshold;

    // Convert limits from degrees to encoder pulses
     this->MIN_POS = degToPulse(minPos_degrees);
     this->MAX_POS = degToPulse(maxPos_degrees);

    // Initialize motion contrainsts to defaults
     this->cmdSpeed = defaultSpeed;
     this->accel = defaultAccel;
     this->dir_speed =  FORWARD;
     this->dir_accel =  FORWARD;
    
    // Initialize state variables to zero
     this->currSpeed = 0;
     this->encoderSpeed = 0;
     this->encoderAccel = 0;
     this->currPos = 0;
     this->cmdPos = 0;
}

// Required to allow motors to move.
// Must be called when controller restarts and after any error.
void Motor::exitSafeStart() {
  Wire.beginTransmission(this->I2C_NUM);
  Wire.write(0x83);  // Exit safe start
  Wire.endTransmission();
}

// Set motor speed and direction
// INPUT: Motor speed [between 0 and 3200]
void Motor::sendMotorSpeed(int16_t speed) {
  uint8_t cmd = 0x85;  // Motor forward
  if (speed < 0)  {
    cmd = 0x86;  // Motor reverse
    speed = (int16_t) abs(speed);
  }

  Wire.beginTransmission(this->I2C_NUM);
  Wire.write(cmd);
  Wire.write(speed & 0x1F);
  Wire.write(speed >> 5 & 0x7F);
  Wire.endTransmission();
}
 
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

// Sets the new desired position of the motor after checking if within limits
// INPUT: Desired position [in degrees]
void Motor::setPosition(float newPos_degrees) {
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

  this->setDirection(this->cmdPos);
}

// Checks which direction to go based on provided position
// INPUT: position [in encoder pulses]
void Motor::setDirection(float newPos) {
  // Determine which direction to go
  if (this->currPos > newPos) {
    this->dir_speed = this->REVERSE;
    this->dir_accel = this->REVERSE;
  } else {
    this->dir_speed = this->FORWARD;
    this->dir_accel = this->FORWARD;
  }
  //print();
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

// Send a modified speed to controller based on max positions
// INPUT: percent of max motor speed [-100 to 100]
void Motor::moveMotor(float newPercent) {
  float deltaPos = 0;
  float nPos = 0;
  float percent_speed = 0;

  //Calculate the difference between the current position and the endstop threshold which the arm would be moving towards
  if(newPercent > 0){
    deltaPos = (this->getMaxPos() - this->getMoveThreshold()) - this->getCurrPos(); //neg if within threshold, otherwise positive distance
  } else if (newPercent < 0) {
    deltaPos = this->getCurrPos() - (this->getMinPos() + this->getMoveThreshold()); //neg if within threshold, otherwise positive distance
  } else {
    deltaPos = -1 * this->getMoveThreshold();  // 0 Speed
  }

  //If negative, can go full speed, otherwise calculate a cosine interpolation speed
  if(deltaPos > 0){
    nPos = PI/2;
  } else {
    nPos = map(abs(deltaPos), 0, this->getMoveThreshold(), PI/2, 0);
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
  this->currSpeed = percent_speed / 100 * this->cmdSpeed;
  this->sendMotorSpeed(currSpeed);
}

// Apply the motor acceleration using Newton's 2nd Law of Motion towards desired position
float Motor::applyPID(int timer) {

  // SAFETY LIMITS
  // If motor is beyond max safe limits, arm is stationary until reset or manually homed
  if (this->currPos > this->MAX_POS*1.1 || this->currPos < this->MIN_POS*1.1) {
    this->currSpeed = 0;
    return currSpeed;
  }
/* 
 -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   PID CONTROLLER GOES HERE
 -----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 */
  
 
  return currSpeed;
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

void Motor::reset() {
  this->currPos = 0;
  this->cmdPos = 0;
  this->currSpeed = 0;
  this->exitSafeStart();
}

void Motor::print() {
  Serial.print(this->currPos);
  Serial.print(",");
  Serial.print(this->cmdPos);
  Serial.print(",");
  Serial.print(this->currSpeed);
  Serial.print(",");
  Serial.println(this->cmdSpeed);
  /*
  Serial.print(",");
  Serial.print(this->dir_speed);
  Serial.print(",");
  Serial.println(this->dir_accel);
  */
}

//Getter funcs for private vars
float Motor::getCurrPos() {return currPos;}
float Motor::getCmdPos() {return cmdPos;}
float Motor::getMoveThreshold() {return MOVE_THRESHOLD;}
float Motor::getMinPos() {return MIN_POS;}
float Motor::getMaxPos() {return MAX_POS;}

//Calculate encoder Count from degrees
float Motor::degToPulse(float deg)  {
  return deg / 360 * this->PULSE_PER_REV * this->GEAR_RATIO;
}
