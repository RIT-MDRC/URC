#include "Motor.h"

Motor::Motor(float minPos_degrees, float maxPos_degrees, double pulses, double ratio, uint8_t deviceNum, float defaultSpeed, float defaultAccel, float moveThreshold) {
    // MOTOR CONSTANTS
     this->PULSE_PER_REV = pulses;
     this->GEAR_RATIO = ratio;
     this->I2C_NUM = deviceNum;   // I2C Device # of motor controller

    // Convert limits from degrees to encoder pulses
     this->MIN_POS = minPos_degrees / 360 * pulses * ratio;
     this->MAX_POS = minPos_degrees / 360 * pulses * ratio;
     this->MOVE_THRESHOLD = moveThreshold;

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
     this->stopDist = 0;
}

// Required to allow motors to move.
// Must be called when controller restarts and after any error.
void Motor::exitSafeStart() {
  Wire.beginTransmission(this->I2C_NUM);
  Wire.write(0x83);  // Exit safe start
  Wire.endTransmission();
}

// Set motor speed (between 0 and 3200) and direction
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

//Send a modified speed to controller based on max positions
void Motor::moveMotor(int16_t speed) {
  float dPos = 0;
  float nPos = 0;
  int16_t nSpeed = 0;

  if(speed > 0){
    dPos = (getMaxPos() - getMoveThreshold()) - getCurrPos(); //neg if within threshold, otherwise positive distance
  }
  else if(speed < 0){
    dPos = getCurrPos() - (getMoveThreshold() - getMinPos()); //neg if within threshold, otherwise positive distance
  }
  else{
    dPos = -1*getMoveThreshold();  // 0 Speed
  }

  if(dPos > 0){
    nPos = PI/2;
  }
  else{
    nPos = map(abs(dPos), 0, getMoveThreshold(), PI/2, 0);
  }
  
  int16_t maxSpeed = (-cos(nPos) + 1) * 100 * (speed / abs(speed));

  if(speed > maxSpeed){
    nSpeed = maxSpeed;
  }
  else{
    nSpeed = speed;
  }

  this->sendMotorSpeed(nSpeed);
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

void Motor::setPosition(float newPos_degrees) {
  // Convert desired position from degrees to encoder pulses
  float newPos = newPos_degrees / 360 * this->PULSE_PER_REV * this->GEAR_RATIO;

  // Check if desired position is within safe limits
  if (newPos <= this->MAX_POS || newPos >= this->MIN_POS) {
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
    return;
  }
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

bool Motor::setMaxSpeed(float newSpeed) {
  if (abs(newSpeed) <= this->ABS_MAX_SPEED) {
     this->cmdSpeed = abs(newSpeed);
    return true;
  }
  return false;
}

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

  // Calculate the minimum distance (in encoder pulse) needed to stop
  // Based on V_f^2 = V_i^2 + 2*a*s, where V_f = 0
  if (this->encoderAccel != 0) {
    this->stopDist = (pow(this->encoderSpeed,2)) / (2*abs(this->encoderAccel));
  } else {this->stopDist = 0;}
}

float Motor::applyAccel() {
  // Check if motor is close enough to position
  if (abs(this->currPos - this->cmdPos) >= 10) {
    
    // Check if motor has overshot desired position and reverse direction if needed
    if (this->currPos > this->cmdPos && this->dir_speed == this->FORWARD) {this->setPosition(this->cmdPos);Serial.println("WORKS");}
    if (this->currPos < this->cmdPos && this->dir_speed == this->REVERSE) {this->setPosition(this->cmdPos);Serial.println("MAYBE WORKS");}
    
    // Calculate new speed after acceleration and put it in buffer
    float buf_speed = this->currSpeed + (this->accel * (float)this->dir_accel);
    // Check if new speed is less than desired max speed
    if (abs(buf_speed) <= this->cmdSpeed) {
      // If going forward, check if the new speed is positive (and above min)
      if (this->dir_speed == this->FORWARD) {
        if (buf_speed <= this->ABS_MIN_SPEED) {this->currSpeed = buf_speed;}
        // If speed is less than minimum speed, increase it
        else {this->currSpeed += this->accel * 2;}
      }
      // If going in reverse, check if the new speed is negative (and below min)
      if (this->dir_speed == this->REVERSE) {
        if (-buf_speed >= this->ABS_MIN_SPEED) {this->currSpeed = buf_speed;}
        // If speed is more than minimum speed, decrease it
        else {this->currSpeed -= this->accel * 2;}
      }
      if(abs(this->currSpeed) > this->cmdSpeed + this->accel) {
        this->currSpeed = buf_speed + this->accel*this->dir_speed;
        Serial.println("EDGE CASE");
      }
    }

/*
    // Check if motor is slowing down too soon and won't make it to position
    if (abs(currPos - cmdPos) >= 25 && abs(currSpeed) <= (120)) {currSpeed += accel * dir_speed;}
 */
    
    // Determine if getting close to position and start slowing down
    if ( ( (this->dir_speed == this->FORWARD && this->cmdPos - this->currPos < this->stopDist) || (this->dir_speed == this->REVERSE && this->currPos - this->cmdPos < this->stopDist) ) && (this->dir_speed == this->dir_accel) ) 
      {this->dir_accel *= -1;}
    
    return this->currSpeed;
  }
  this->currSpeed = 0;
  return 0;
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
  Serial.print(this->encoderSpeed);
  Serial.print(",");
  Serial.print(this->cmdSpeed);
  Serial.print(",");
  Serial.print(this->dir_speed);
  Serial.print(",");
  Serial.println(this->dir_accel);
}

//Getter funcs for private vars
float Motor::getCurrPos() {
  return currPos;
}

float Motor::getCmdPos()  {
  return cmdPos;
}

//Calculate encoder Count from degrees
float Motor::degToPulse(float deg)  {
  return deg / 360 * this->PULSE_PER_REV * this->GEAR_RATIO;
}

float Motor::getMoveThreshold()  {
  return MOVE_THRESHOLD;
}

float Motor::getMinPos() {
  return MIN_POS;
}

float Motor::getMaxPos() {
  return MAX_POS;
}
