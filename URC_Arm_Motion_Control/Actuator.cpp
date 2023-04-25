#include "Actuator.h" 

// CONSTRUCTOR
// INPUTS: I2C Device Numcer, Subtype of Actuator, Minimum Position, Maximum Position, Default Maximum Safe Speed, Acceleration, Encoder units per Revolution,
//            Gear Ratio of Gearbox between Actuator Output and Joint, Should the direction of travel be reversed?
Actuator::Actuator(uint8_t device_num, ActuatorType type, float minPos_degrees, float maxPos_degrees, float max_speed, float acc, double units_per_rev, double gear_ratio, bool reversed) {
  // Initialize unchangable constants
  this->I2C_NUM = device_num;
  this->ACTUATOR_TYPE = type;
  this->UNITS_PER_REV = units_per_rev;
  this->GEAR_RATIO = gear_ratio;
  this->MIN_POS = degToUnit(minPos_degrees);
  this->MAX_POS = degToUnit(maxPos_degrees);

  // Initialize motion limits
  this->sendSafeMaxSpeed(max_speed);
  this->accel = acc;

  // Initialize flag variables
  this->error = ErrorCode::NO_ERROR;
  this->reversedMotion = reversed;
  
  // Initialize state and position algorithm variables to zero
  this->currPos = 0;
  this->cmdPos = 0;
  this->currSpeed = 0;
  this->encoderSpeed = 0;
  this->encoderAccel = 0;

  // Intialize PID Position Algorithm parameters (Gains will be set by another function)
  this->Kp = 1;
  this->Ki = 0;
  this->Kd = 0;
  this->integral = 0;
  this->lastError = 0;
  this->lastPos = 0;
  
  // If using actuator, set error state so that joint doesn't move until position calibrated
  if (type == ActuatorType::LINEAR) {this->error = ErrorCode::POS_UNCERTAIN;}
}

// COMMON ERROR HANDLING -----------------------------------------------------------------------------------------

// Must be called after start-up or reboot to let driver know it is safe to start moving
void Actuator::exitSafeStart() {
  // Clear the error state unless the joint has lost its position
  if (this->error == ErrorCode::POS_UNCERTAIN) {return;}
  else {this->error = ErrorCode::NO_ERROR;}
  
  // Send command to driver
  this->commandQuick(I2CCommand::ExitSafeStart);

  // Tell user which driver the command was sent to
  Serial.print(" Driver ");
  Serial.print(this->I2C_NUM);
  Serial.println(" exited safe start");
}

// Must be called at least once per second to let driver know that everything is still okay
// If command timeout error occurs, an exitSafeStart command must be sent to reset driver
void Actuator::resetCommandTimeout() {
  // MOTOR and LINEAR drivers do not have the feature, so will ignore this command
  if (this->ACTUATOR_TYPE == ActuatorType::MOTOR || this->ACTUATOR_TYPE == ActuatorType::LINEAR) {return;}
  commandQuick(I2CCommand::ResetCommandTimeout);
}

// COMMON MOTION CONTROL -----------------------------------------------------------------------------------------

// Tells driver directly what speed to drive
// INPUT: -100 to 100 % of max safe speed
void Actuator::sendTargetSpeed(double percent) {
  // Exit function if joint is in an error state
  if (this->error != ErrorCode::NO_ERROR) {return;}
  
  // Make sure desired percent is <= 100%
  percent = constrain(percent,-100,100);
  
  // Calculate new speed
  double speed = (double)this->getSafeMaxSpeed() * percent / (double)100;

  // Check if speed needs to be reversed due to hardware mix-up
  if (reversedMotion) {speed *= -1;}
  
  // If joint is a motor/actuator, use the MOTOR/LINEAR specific command
  if (ACTUATOR_TYPE == ActuatorType::MOTOR || ACTUATOR_TYPE == ActuatorType::LINEAR) {this->sendMotorSpeed((float)speed);}
  
  // If joint is a stepper, convert speed to steps per 10000 seconds before sending
  // NOTE: I have no idea why the driver uses steps per 10000 seconds. Thats just what the documentation says
  else if (this->ACTUATOR_TYPE == ActuatorType::STEPPER) {commandW32(I2CCommand::SetTargetVelocity, (int32_t)speed*10000);}

  // Store new speed
  this->currSpeed = (float)speed;
}

// Tells joint a new desired angular position
// INPUT: If motor:    Convert to encoder pulse and calculate path with postion algorithm
//        If actuator: Convert to linear position and calculate path with postion algorithm
//        If stepper:  Convert to steps and send to stepper controller
void Actuator::sendTargetPosition(float newPos_degrees) {
  // Exit function if joint is in an error state
  if (this->error != ErrorCode::NO_ERROR) {return;}
  
  // Convert desired position from degrees to joint native units
  float newPos = this->degToUnit(newPos_degrees);

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
  }
  
  // If motor or actuator, DO SOMETHING SPECIFIC
  if (this->ACTUATOR_TYPE == ActuatorType::MOTOR || ACTUATOR_TYPE == ActuatorType::LINEAR) {
    // PUT MOTOR/LINEAR SPECIFIC CODE HERE
  } 
  // If stepper, joint will be able to reach position on its own
  else if (this->ACTUATOR_TYPE == ActuatorType::STEPPER) {
    commandW32(I2CCommand::SetTargetPosition, (int32_t)newPos);
  }

  // If joint doesn't have encoder, store new position value
  if (!this->hasEncoder) {this->currPos = newPos;}
}

// Tells joint a new desired max speed (NOTE: This is seperate from the ABSOLUTE maximum speed)
void Actuator::sendSafeMaxSpeed(float newMax) {
  // Make sure input value positive
  newMax = abs(newMax);
  // Make sure input value is within absolute limits of actuator type
  if ((this->ACTUATOR_TYPE == ActuatorType::MOTOR || this->ACTUATOR_TYPE == ActuatorType::LINEAR) && newMax > Actuator::MOTOR_ABS_MAX_SPEED) {
    newMax = Actuator::MOTOR_ABS_MAX_SPEED;
  } else if (this->ACTUATOR_TYPE == ActuatorType::STEPPER && newMax > Actuator::STEPPER_ABS_MAX_SPEED) {
    newMax = Actuator::STEPPER_ABS_MAX_SPEED;
  }
  
  // If stepper, speed limit is enforced by the driver
  if (this->ACTUATOR_TYPE == ActuatorType::STEPPER) {commandW32(I2CCommand::SetSpeedMax, this->safeMaxSpeed);}
  // If motor/linear, speed limit is enforced by position algorithm

  // Store new maximum speed
  this->safeMaxSpeed = newMax;
}

// Determine velocity and accelerations [in native units per code loop] based on encoder readings
// INPUT: new position reading of the encoder
void Actuator::interpretEncoder(float newPos) {
  // If joint has no encoder, exit function
  if (!hasEncoder) {return;}
  
  // Before calculations, store speed from last loop
  float last_encoderSpeed = this->encoderSpeed;
  // Calculate new speed (change since last loop)
  this->encoderSpeed = newPos - this->currPos;
  this->encoderAccel = this->encoderSpeed - last_encoderSpeed;
  // Store encoder reading
  this->currPos = newPos;
}

// Tell joint to keep moving in one direction until it reaches limit switch (used to recalibrate position)
void Actuator::goHome(int dir) {
   // COMPLETE AFTER LIMIT SWITCHES ARE INSTALLED
}

// MOTOR SPECIFIC FUNCTIONS -------------------------------------------------------------------------------------

// Determines which I2C command to send based on input speed
// INPUT: desired speed [0 to 3200]
void Actuator::sendMotorSpeed(double newSpeed) {
  // Initialize motor to go forward
  I2CCommand cmd = I2CCommand::MotorForward;

  // If input value is negative, change command to reverse and make value positive
  if (newSpeed < 0)  {
    cmd = I2CCommand::MotorReverse;
    newSpeed = abs(newSpeed);
  }
  
  // Send the command
  this->commandW12(cmd, (uint16_t)newSpeed);
}

// Set new gains for PID position algorithm
// INPUT: desired gains
void Actuator::setPIDGains(double P, double I, double D) {
  this->Kp = P;
  this->Ki = I;
  this->Kd = D;
}

// Use calculated path to move joint to specified position
// INPUT: Time passed from last loop (for integral calculation)
void Actuator::positionAlgorithm(int dT) {
  // Only run algorithm for motors/actuators and if not in an error state;
  if (!(this->ACTUATOR_TYPE == ActuatorType::MOTOR || this->ACTUATOR_TYPE == ActuatorType::LINEAR)) {return;} 
  if (this->error != ErrorCode::NO_ERROR) {return;}

  // Calculate difference between desired and current position
  double error = this->cmdPos - this->currPos;
  // Calculate area under position curve
  integral += (error + this->lastError) / 2 * (double)dT;   //Riemann sum integral
  // Calculate derivate of position
  double dPos = (double)(this->currPos - this->lastPos) / (double)dT;   //derivative
  // Store position difference for next loop
  this->lastError = error;
  this->lastPos = this->currPos;
  // Do PID calculation
  double PID = (this->Kp * error) + (this->Ki * this->integral) - (this->Kd * dPos);

  // Send speed to joint
  this->sendTargetSpeed(constrain(PID, -100, 100));
}

// LINEAR ACTUATOR SPECIFIC FUNCTIONS ----------------------------------------------------------------------------------

// Recalibrates stored position value based on absolute encoder of encoder
// INPUT: Encoder reading in native units of joint
void Actuator::calibratePos(float pos) {
  // If not an actuator, joint doesn't have an absolute encoder and shouldnt use this function
  if (this->ACTUATOR_TYPE != ActuatorType::LINEAR) {return;}
  
  // Store current position of encoder
  this->currPos = pos;
  this->cmdPos = pos;
  // Clear uncertain position error state
  this->error = ErrorCode::NO_ERROR;

  this->exitSafeStart();
}

// MISC FUNCTIONS -----------------------------------------------------------------------------------------------

// Prints out state variables and current error state onto Serial Terminal
void Actuator::print() {
  Serial.print(" currPos = ");
  Serial.print(this->currPos);
  Serial.print(", cmdPos = ");
  Serial.print(this->cmdPos); // NOTE: If actuator has no encoder, currPos always equals cmdPos
  Serial.print(", currSpeed = ");
  Serial.print(this->currSpeed);  // NOTE: If actuator is stepper, currSpeed will equal safeMaxSpeed unless manually driving
  Serial.print(", safeMaxSpeed = ");
  Serial.println(this->safeMaxSpeed);
  Actuator::printError(this->error);
}

void Actuator::printError(ErrorCode err) {
  Serial.print(" ERROR: ");
  switch (err) {
    case ErrorCode::NO_ERROR:
      Serial.println("No Error");
      break;
    case ErrorCode::USER_INTERUPTION:
      Serial.println("User has interupted actuator motion (RESET to clear)");
      break;
    case ErrorCode::POS_LIMIT_REACHED:
      Serial.println("Safe Position Limit has been exceeded (HOME to clear)");
      break;
    case ErrorCode::POS_UNCERTAIN:
      Serial.println("Actuator Position is Uncertain (HOME to clear)");
      break;
    default:
      Serial.println("Unknown Error");
      break;
  }
}

// Stops motor, clears error state, resets I2C communication line, clears all state variables, and set new current position
// INPUT: Current encoder reading (in native units)
void Actuator::reset(float pos) {
  // Store current encoder reading and set it as desired position so actuator wont try to move
  this->currPos = pos;
  this->cmdPos = this->currPos;

  // Set current speed to zero
  this->currSpeed = 0;

  // If actuator is stepper, use Halt and Set Position command
  if (ACTUATOR_TYPE == ActuatorType::STEPPER) {this->commandW32(I2CCommand::HaltAndSetPosition, pos);}
  
  // If actuator is motor/linear, use brake command then recalibrate position algorithm to current position 
  else if (ACTUATOR_TYPE == ActuatorType::MOTOR || ACTUATOR_TYPE == ActuatorType::LINEAR) {
    this->commandW7(I2CCommand::MotorBrake, 16);

    // Clear PID variables
    this->integral = 0;
    this->lastError = 0;
  }
  
  // Give driver some time to execute command
  delay(5);
  
  // Clear error state unless it is a position uncertain error (must HOME or RECALIBRATE to clear that error)
  if (error !=ErrorCode::POS_UNCERTAIN) {
    this->error = ErrorCode::NO_ERROR;
    // Let driver know it is now safe to move again
    this->exitSafeStart();
  }
  
  // Print message to Serial Terminal to let user know which driver was reset
  Serial.print("RESET Device ");
  Serial.println(this->I2C_NUM);
}
// If no position given, set current position to zero
void Actuator::reset() {this->reset(0);}

// Stops the motor and raises error flag (requires reset to remove)
void Actuator::stop(ErrorCode err)  {
  // If stepper, use halt and hold command
  if (ACTUATOR_TYPE == ActuatorType::STEPPER) {this->commandQuick(I2CCommand::HaltAndHold);}
  
  // If actuator is motor/linear, use brake command 
  else if (ACTUATOR_TYPE == ActuatorType::MOTOR || ACTUATOR_TYPE == ActuatorType::LINEAR) {this->commandW7(I2CCommand::MotorBrake, 32);}

  // Set error state and print message to Serial Terminal to let user know what went wrong
  this->error = err;
  Actuator::printError(err);
}

// Convert from degrees to native unit of joint (ex: steps or encoder pulses)
float Actuator::degToUnit(float deg) {
  return deg / (float)360 * this->UNITS_PER_REV * this->GEAR_RATIO;
}

// Convert from native unit of joint to degrees for better readability
float Actuator::unitToDeg(float unit) { 
  return unit / this->UNITS_PER_REV / this->GEAR_RATIO * (float)360;
}

// Tells actuator to ignore all encoder functions
void Actuator::noEncoder() {this->hasEncoder = false;}    

// GETTERS AND SETTER --------------------------------------------------------------------------------------------

float Actuator::getCurrPos() {return currPos;}
float Actuator::getCmdPos() {return cmdPos;}
float Actuator::getMinPos() {return MIN_POS;}
float Actuator::getMaxPos() {return MAX_POS;}
float Actuator::getSafeMaxSpeed() {return safeMaxSpeed;}

// I2C COMMAND FUNCTIONS ----------------------------------------------------------------------------------------
// Sends an I2C command with a no parameter
void Actuator::commandQuick(I2CCommand cmd) {
  Wire.beginTransmission(this->I2C_NUM);
  Wire.write((uint8_t)cmd);
  _lastError = Wire.endTransmission();
}

// Sends an I2C command with a 32-bit parameter
void Actuator::commandW32(I2CCommand cmd, uint32_t val) {
  Wire.beginTransmission(this->I2C_NUM);
  Wire.write((uint8_t)cmd);
  Wire.write((uint8_t)(val >> 0)); // lowest byte
  Wire.write((uint8_t)(val >> 8));
  Wire.write((uint8_t)(val >> 16));
  Wire.write((uint8_t)(val >> 24)); // highest byte
  _lastError = Wire.endTransmission();
}

// Sends an I2C command with a 12-bit parameter
void Actuator::commandW12(I2CCommand cmd, uint16_t val) {
  Wire.beginTransmission(this->I2C_NUM);
  Wire.write((uint16_t)cmd);
  Wire.write(val & 0x1F);
  Wire.write(val >> 5 & 0x7F);
  _lastError = Wire.endTransmission();
}

// Sends an I2C command with a 7-bit parameter
void Actuator::commandW7(I2CCommand cmd, uint8_t val) {
  Wire.beginTransmission(this->I2C_NUM);
  Wire.write((uint8_t)cmd);
  Wire.write((uint8_t)(val & 0x7F));
  _lastError = Wire.endTransmission();
}
