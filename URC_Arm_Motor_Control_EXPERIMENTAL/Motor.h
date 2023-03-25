#include "Arduino.h"
#include "Math.h"
#include <Wire.h>
#include <Encoder.h>

class Motor {
  public:
    // CONTRUCTION
    Motor(float minPos_degrees, float maxPos_degrees, double pulses, double ratio, uint8_t deciveNum, float defaultSpeed, float defaultAccel, bool reversed);
    
    // I2C COMMUNICATION
    void exitSafeStart();  // Initialize controller
    void sendMotorSpeed(int16_t speed);  // Send cmdSpeed to controller
    uint16_t readUpTime();
    
    float positionAlgorithm();         // Use path calculated by setNewPosition to move joint to specified position
    void driveMotor(float newPercent); // Manually drive motor by specifying percentage of maximum speed (will limit speed when approaching min/max positions)

    // POSITION CONTROL
    void setNewPosition(float newPos);   // Interpret user input and determine new direction to go
    bool setMaxSpeed(float newSpeed);    // Set the maximum allowable speed of the motor and check if user input is within acceptable bounds
    void interpretEncoder(float newPos); // Take in encoder reading and interpret
    void homing(int16_t dir);            // Home motor joint by moving at a slow speed and ignoring software limits on position.
    float degToPulse(float deg);         // Calculate encoder position from degrees

    // MISC FUNCTIONS
    void print(); // Print state to the serial moniter
    void reset(float pos); // Stops motor, clears error state, resets I2C communication line, clears all state variables, and set new current position
    void reset(); // Reset motor and set new position to zero
    void stop(int errorCode); // Stops the motor and raises error flag (requires reset to remove)

    //Getter functions for private vars.
    float getCurrPos();
    float getCmdPos();
    float getMoveThreshold();
    float getMinPos();
    float getMaxPos();
    
  private:    
    //GLOBAL MOTOR CONSTANTS
    const float ABS_MAX_SPEED = 3200;  // Maximum drivable speed
    const float ABS_MIN_SPEED = 120;   // Minimum drivable speed
    // Const for making direction control more readable
    const int FORWARD = 1;
    const int REVERSE = -1;

    //ERROR CODES
    const int USER_INTERUPTION = 0;     // User told motor to stop
    const int POS_LIMIT_REACHED = 1; // Limits were reached but the motor kept going
    
    // UNCHANGABLE MOTOR CONSTANTS
    double PULSE_PER_REV;  // Encoder pulses per motor shaft revolution
    double GEAR_RATIO;     // Gear ratio of gearbox
    uint8_t I2C_NUM;       // I2C Device # of motor controller
    float MIN_POS;         // Mininum safe position
    float MAX_POS;         // Maxinum safe position

    // CHANGABLE MOTOR CONSTANTS
    float accel;          // Maximum change in speed per clock cycle
    float cmdSpeed;       // Max speed commanded by user
    
    // STATE VARIABLES
    float currSpeed;      // Speed being sent to motor controller
    float currPos;        // Current position of the motor
    float cmdPos;         // Position commanded by user

    // ENCODER VARIABLES
    float encoderSpeed;   // Actual speed of motor
    float encoderAccel;   // Actual accel of motor
    
    // POSITION ALGORITHM VARIABLES
    int dir_travel;            // Which way is motor trying to go? (Which way is accelerate applied?)
    int dir_initialSpeed;      // Which way was the motor going when path was calculated?
    float delta_pos_SafeStop;  // Distance required to safely stop at desired position (used in path calculation)
    float initialPos;          // inital motor position when path was calculated

    // FLAG VARIABLES
    bool error;             // Is motor in an error state and unsafe to run?
    bool reversedMotion;    // Is the hardware making the motor go the wrong way? (Can fix hardware instead)
    bool overshooting;      // Is the initial speed to large to reach desired position without overshooting?
    bool started_opposite;  // Is the initial speed direction opposite to the travel direction from initial to desired positions
};
