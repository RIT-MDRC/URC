#include "Arduino.h"
#include "Math.h"

class Joint {
  public:
    //CONSTRUCTOR
    Joint(char joint_num, int pot_pin, int min_pot_val, int max_pot_val, int min_joint_pos, int max_joint_pos, bool reverse);
    
    //MISC FUNCTIONS
    // Initialize potentiometer input pin
    void init();
    // Read potentiometer value and convert to joint position
    void read();

    //OUTPUT
    // Print joint position to Serial Terminal 
    void print();
    // Create G-CODE for sending to big arm
    String gcode();

    //GETTERS
    // State Variables
    int getPotValue();
    int getJointPos();
    // Constants
    int getMinPotValue();
    int getMaxPotValue();
    int getMinJointPos();
    int getMaxJointPos();
    int getPotPin();
    char getJointNum();
    bool isReversed();
    
  private:
    //SETTERS
    void setPotValue(int val);
    void setJointPos(int pos);
    
    //CONSTANTS
    int MAX_JOINT_POS;   // Maximum position on big arm for corrisponding joint
    int MIN_JOINT_POS;   // Minimum position on big arm for corrisponding joint
    int MAX_POT_VALUE;   // Maximum potentiometer reading
    int MIN_POT_VALUE;   // Minimum potentiometer reading
    int POT_PIN;         // Analog input pin for potentiometer
    int JOINT_NUM;       // Number code of joint [1-6] for creating GCODE and printing to terminal
    bool REVERSED;       // Does the potentiometer reading go opposite direction of joint position?

    //STATE VARIABLES
    int pot_value;    // Current potentiometer reading
    int joint_pos;    // Conversion from potentiometer reading to position of big arm
};
