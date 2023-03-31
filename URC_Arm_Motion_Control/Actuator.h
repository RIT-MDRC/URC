#include "Arduino.h"
#include "Math.h"
#include <Wire.h>
#include <AutoPID.h>

//ACTUATOR TYPES
// These determine which I2C commands to use and how position is controlled
enum class ActuatorType {
  MOTOR    = 0,
  STEPPER  = 1,
  LINEAR   = 2,
};

//DIRECTION CONSTANTS
// Makes code more readable
enum class Direction {
  FORWARD =  1,
  REVERSE = -1,
};

//ERROR CODES
// Makes code more readable
enum class ErrorCode {
  NO_ERROR          = 0,
  USER_INTERUPTION  = 1,   // User told motor to stop
  POS_LIMIT_REACHED = 2,   // Limits were reached but the motor kept going
  POS_UNCERTAIN     = 3,   // When using absolute encoder or no encoder, position has been lost
  // NOTE: POS_UNCERTAIN can only be cleared by homing or recalibrating position
};

// I2C COMMANDS
// These are sent at the beginning of the communication after the device number
enum class I2CCommand {
  //STEPPER COMMANDS
  SetTargetPosition                 = 0xE0,
  SetTargetVelocity                 = 0xE3,
  HaltAndSetPosition                = 0xEC,
  HaltAndHold                       = 0x89,
  GoHome                            = 0x97,
  ResetCommandTimeout               = 0x8C,
  Deenergize                        = 0x86,
  Energize                          = 0x85,
  ExitSafeStart                     = 0x83,
  EnterSafeStart                    = 0x8F,
  Reset                             = 0xB0,
  ClearDriverError                  = 0x8A,
  SetSpeedMax                       = 0xE6,
  SetStartingSpeed                  = 0xE5,
  SetAccelMax                       = 0xEA,
  SetDecelMax                       = 0xE9,
  SetStepMode                       = 0x94,
  SetCurrentLimit                   = 0x91,
  SetDecayMode                      = 0x92,
  SetAgcOption                      = 0x98,
  GetVariable                       = 0xA1,
  GetVariableAndClearErrorsOccurred = 0xA2,
  GetSetting                        = 0xA8,

  //MOTOR COMMANDS
  // Linear Actuator will also use these commands
  MotorForward                      = 0x85,
  MotorReverse                      = 0x86,
  MotorBrake                        = 0x92,
  SetMotorLimit                     = 0xA2,
  StopMotor                         = 0xE0,
};

class Actuator {
  public:
    // CONTRUCTION
    Actuator(uint8_t decive_num, ActuatorType type, float minPos_degrees, float maxPos_degrees, float max_speed, float acc, double units_per_rev, double gear_ratio, bool reversed);

    // COMMON FUNCTIONS
    // Error Handling
    void exitSafeStart();                 // Tell driver it is safe to start motor after reboot/reset
    void resetCommandTimeout();            // If not sent 1/sec, driver will get command timout error
    // Motion Control
    void sendTargetSpeed(double percent);            // Used for speed control of joint
    void sendTargetPosition(float newPos_degrees);   // Used for position control of joint
    void sendSafeMaxSpeed(float newMax);             // Set the maximum allowable speed of the joint
    void interpretEncoder(float newPos);             // Take in encoder reading and interpret
    void goHome(int dir);                            // Recalibrate home/zero positon of joint

    // MOTOR CONTROL (Control position by adjsuting speed)
    void sendMotorSpeed(double newSpeed);      // Send speed to motor controller in its perferred format
    void setPIDGains(double P, double I, double D); // Set parameters of PID position algorithm
    void positionAlgorithm(int dT);                  // Execute pre-calculated path to desired position by controlling speed
    
    // LINEAR_ACTUATOR CONTROL
    void calibratePos(float pos);
    
    // STEPPER CONTROL (Control position by stepping a certain distance)
    void setStepMode(int);      // Set stepping mode (defaults to full steps)
    void setCurrentLimit(int);  // Set current limit of motor (defaults to 480 mA)
    void getOperationState();   // Check if the driver has any internal errors 

    // MISC FUNCTIONS
    void print();                   // Print joint state variables to the serial moniter
    void printError(ErrorCode err); // Prints out descriptive message of error based on specified error code
    void reset(float pos);          // Stops motor, clears error state, resets I2C communication line, clears all state variables, and set new current position
    void reset();                   // Reset motor and set new position to zero
    void stop(ErrorCode err);       // Stops the motor and raises error flag (requires reset to remove)
    float degToUnit(float deg);     // Convert from degrees to native unit of joint (ex: steps or encoder pulses)
    float unitToDeg(float unit);    // Convert from native unit of joint to degrees for better readability 
    void noEncoder();               // After calling, all encoder functions for joint will be ignored

    //Getter functions for private vars.
    float getCurrPos();
    float getCmdPos();
    float getMinPos();
    float getMaxPos();
    float getSafeMaxSpeed();

  private:
    //I2C COMMUNICATION
    void commandQuick(I2CCommand cmd);             // Send a command with no parameter
    void commandW32(I2CCommand cmd, uint32_t val); // Send a command with a 32-bit parameter
    void commandW12(I2CCommand cmd, uint16_t val);  // Send a command with a 12-bit parameter
    void commandW7(I2CCommand cmd, uint8_t val);   // Send a command with a 7-bit parameter
    uint8_t _lastError = 0;     // If there is an communication error, a value will be returned by the controller
        
    // UNCHANGABLE ACTUATOR CONSTANTS
    float MOTOR_ABS_MAX_SPEED = 3200;  // Maximum drivable speed
    float MOTOR_ABS_MIN_SPEED = 120;   // Minimum drivable speed
    float STEPPER_ABS_MAX_SPEED = 200;  // Maximum drivable speed
    float STEPPER_ABS_MIN_SPEED = 20;   // Minimum drivable speed
    
    double UNITS_PER_REV;   // Encoder pulses/stepper steps per shaft revolution
    double GEAR_RATIO;     // Gear ratio of gearbox
    ActuatorType ACTUATOR_TYPE;  // Identifier for type of joint control scheme
    uint8_t I2C_NUM;       // I2C Device # of motor controller
    float MIN_POS;         // Mininum safe position (in native units)
    float MAX_POS;         // Maxinum safe position (in native units)

    // CHANGABLE MOTOR CONSTANTS
    float accel;          // Maximum change in speed per clock cycle
    float safeMaxSpeed;   // Max speed the user thinks is safe
    
    // STATE VARIABLES
    float currSpeed;      // Speed being sent to motor controller
    float currPos;        // Current position of the motor
    float cmdPos;         // Position commanded by user

    // ENCODER VARIABLES
    float encoderSpeed;   // Actual speed of motor
    float encoderAccel;   // Actual accel of motor
    
    // POSITION ALGORITHM VARIABLES
    double Kp;     // Proportional gain for PID algorithm
    double Ki;     // Integral gain for PID algorithm
    double Kd;     // Derivative gain for PID algorithm
    double integral;   // Value of integral under position curve for PID
    double lastError;  // Used to store last loop's difference between desired and current position
    
    // FLAG VARIABLES
    ErrorCode error;        // Is motor in an error state and unsafe to run?
    bool reversedMotion;    // Is the hardware making the motor go the wrong way? (Can fix hardware instead)
    bool hasEncoder = true; // Does joint have an encoder to read its position?
};
