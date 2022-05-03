#include "Arduino.h"
#include "Math.h"
#include <Wire.h>
#include <Encoder.h>

class Motor {
  public:
    // CONTRUCTION
    Motor(float minPos_degrees, float maxPos_degrees, double pulses, double ratio, uint8_t deciveNum, float defaultSpeed, float defaultAccel, float moveThreshold);
    
    // I2C COMMUNICATION
    void exitSafeStart();  // Initialize controller
    void sendMotorSpeed(int16_t speed);  // Send cmdSpeed to controller
    uint16_t readUpTime();
    
    void moveMotor(float speed); //Send a modified speed to controller based on max positions
    
    // POSITION CONTROL
    void setPosition(float newPos);      // Interpret user input and determine new direction to go
    void setDirection(float newPos);     // Based on desired position, which way do we go?
    bool setMaxSpeed(float newSpeed);    // Set the maximum allowable speed of the motor and check if user input is within acceptable bounds
    void interpretEncoder(float newPos); // Take in encoder reading and interpret
    void homing(int16_t dir);            // Home motor joint by moving at a slow speed and ignoring software limits on position.
    float degToPulse(float deg);         // Calculate encoder position from degrees

    // PID FUNCTIONS
    float applyPID(int timer);                  // Apply PID to motor speed
    void setPIDTuning(int Kp, int Ki, int Kd);  // Set coefficients for PID controller
    
    // MISC FUNCTIONS
    void print(); // Print state to the serial moniter
    void reset(); // Reset I2C communication with driver

    //Getter functions for private vars.
    float getCurrPos();
    float getCmdPos();
    float getMoveThreshold();
    float getMinPos();
    float getMaxPos();
    
  private:    
    //GLOBAL MOTOR CONSTANTS
    float ABS_MAX_SPEED = 3200;  // Maximum drivable speed
    float ABS_MIN_SPEED = 120;   // Minimum drivable speed
    // Const for making direction control more readable
    int FORWARD = 1;
    int REVERSE = -1;
    
    // SPECIFIC MOTOR CONSTANTS
    double PULSE_PER_REV;  // Encoder pulses per motor shaft revolution
    double GEAR_RATIO;     // Gear ratio of gearbox
    uint8_t I2C_NUM;       // I2C Device # of motor controller
    float MIN_POS;         // Mininum safe position
    float MAX_POS;         // Maxinum safe position
    float MOVE_THRESHOLD;  // Distance in degrees from endstops for deceleration of joint

    // MOTOR VARIABLES
    float currSpeed;      // Speed being sent to motor controller
    float cmdSpeed;       // Max speed commanded by user
    
    float accel;    // Maximum change in speed per clock cycle
    
    float encoderSpeed;   // Actual speed of motor
    float encoderAccel;   // Actual accel of motor
    
    float currPos;        // Current position of the motor
    float cmdPos;         // Position commanded by user
    float offsetPos;      // Offset from real position and position stored in encoder object
    
    int dir_speed;    // Which way is motor going?
    int dir_accel;    // Which way is motor trying to go?
    
    // PID Variables
    int Kp, Ki, Kd;
};
