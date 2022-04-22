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

    void moveMotor(int16_t speed); //Send a modified speed to controller based on max positions
    
    uint16_t readUpTime();
    // POSITION CONTROL
    void setPosition(float newPos);    // Interpret user input and determine new direction to go
    bool setMaxSpeed(float newSpeed);  // Check if user input is within acceptable bounds
    void interpretEncoder(float newPos); // Take in encoder reading and interpret
    float applyAccel(); // Speed control of motor based on Newton's 2nd Law
    // MISC FUNCTIONS
    void print(); // Print state to the serial moniter
    void reset(); // Reset I2C communication with driver

    //Getter functions for private vars.
    float getCurrPos();
    float getCmdPos();
    float degToPulse(float deg); //Calculate encoder position from degrees
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
    float MOVE_THRESHOLD;  //distance in degrees from endstops for deceleration of joint

    // MOTOR VARIABLES
    float currSpeed;      // Speed being sent to motor controller
    float cmdSpeed;       // Max speed commanded by user
    float accel;    // Maximum change in speed per clock cycle
    float encoderSpeed;   // Actual speed of motor
    float encoderAccel;   // Actual accel of motor
    float currPos;        // Current position of the motor
    float cmdPos;         // Position commanded by user
    int dir_speed;    // Which way is motor going?
    int dir_accel;    // Which way is motor trying to go?
    float stopDist;   // How long will it take to motor come to a halt based on current accel
};
