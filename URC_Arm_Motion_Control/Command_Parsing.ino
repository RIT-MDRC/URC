/* Read input from Serial Terminal and interpret it

  COMMAND FORMAT: Cn v
  C - Command Identifier (char)
  n - Joint Number (1,2,3,4,5,6)
  v - Command Value (int)
  
 */

// Positions of joints 5 and 6 for use in differential mechanism calculations
float J5_pos = 0;
float J6_pos = 0;

// Speed of joints 5 and 6 for use in differential mechanism calculations
float J5_speed = 0;
float J6_speed = 0;

void parseCommand() {
    // Initialize command value
    float cmdValue = 0;

    // Read command identifier
    char cmd = (char) Serial.read();
    int n = Serial.parseInt() - 1;

    // Convert lower-case letters to upper-case by manupulating ASCII value of char
    // Capital letters always have ASCII value 32 lower than lower-case letters
    if (cmd > 90) {cmd -= 32;}
    
    switch (cmd) {
// --------------------------------------------------------------------------------------------------------------------------------------------------
      case 'P':
        // Set new command position for automatic position control
        cmdValue = Serial.parseFloat();

        // Only use this command if automatic position control is active
        if (!automaticMode) {break;}

        // If setting position of joint 5 or 6, use equation for differential mechanism to calculate needed position of actuators 5 and 6
        // DiffA Act Pos = Joint6 + Joint5
        // DiffB Act Pos = Joint6 - Joint5
        if (n == 4) {
          J5_pos = cmdValue;
          actuator[4].sendTargetPosition(J6_pos + J5_pos);
          actuator[5].sendTargetPosition(J6_pos - J5_pos);
        } else if (n == 5) {
          J6_pos = cmdValue;
          actuator[4].sendTargetPosition(J6_pos + J5_pos);
          actuator[5].sendTargetPosition(J6_pos - J5_pos);
        } 
        
        // Otherwise, just tell actuator its new target position directly
        else {
          actuator[n].sendTargetPosition(cmdValue);
        }
        
        Serial.print("POSITION: ");
        Serial.println(cmdValue);
        break;
// --------------------------------------------------------------------------------------------------------------------------------------------------
      case 'S':
        // Set new desired speed for manual speed control
        cmdValue = Serial.parseFloat();
        
        // Only use this command if automatic position control is not active (ie manual speed control is being used)
        if (automaticMode) {break;}

        // If setting position of joint 5 or 6, use equation for differential mechanism to calculate needed position of actuators 5 and 6
        // DiffA Act Pos = Joint6 + Joint5
        // DiffB Act Pos = Joint6 - Joint5
        if (n == 4) {
          J5_speed = cmdValue;
          actuator[4].sendTargetSpeed(J6_speed + J5_speed);
          actuator[5].sendTargetSpeed(J6_speed - J5_speed);
        } else if (n == 5) {
          J6_speed = cmdValue;
          actuator[4].sendTargetSpeed(J6_speed + J5_speed);
          actuator[5].sendTargetSpeed(J6_speed - J5_speed);
        }
        
        else {actuator[n].sendTargetSpeed(cmdValue);}

        Serial.print("SPEED: ");
        Serial.println(cmdValue);
        break;
// --------------------------------------------------------------------------------------------------------------------------------------------------
      case 'R':
        // Reset specified actuator to clear error or immediately stop motion
        cmdValue = Serial.parseFloat();

        // If command value is non-zero, actuator forgets its position
        // Otherwise, store position of actuator
        if (cmdValue != 0) {
          // Reset actuator
          actuator[n].reset();
          // Reset encoder position or reclaibrate absolute encoder
          switch (n) {
            case 0:
              enc_J1.write(0);
              break;
            case 1:
              enc_J2.write(0);
              break;
            case 2:
              actuator[n].calibratePos(analogRead(J3FeedbackPin));
              break;
            default:
              break;
          }
        }
        // Otherwise, reset and store current position
        else {actuator[n].reset( actuator[n].getCurrPos() );}
        break;
// --------------------------------------------------------------------------------------------------------------------------------------------------
      case 'M':
        // Set new maximum safe speed of specified actuator
        cmdValue = Serial.parseFloat();
        actuator[n].sendSafeMaxSpeed(cmdValue);
        
        Serial.print("SPEED LIMIT: ");
        Serial.println(cmdValue);
        break;
// --------------------------------------------------------------------------------------------------------------------------------------------------
      case 'Q':
        // Print status of actuator
        Serial.print("ACTUATOR ");
        Serial.print(n+1);
        Serial.print(": ");
        actuator[n].print();
        break;
// --------------------------------------------------------------------------------------------------------------------------------------------------
      case 'K':
        // Set gains for PID algorithms
        double Kp = Serial.parseFloat();
        double Ki = Serial.parseFloat();
        double Kd = Serial.parseFloat();

        Serial.print(" Kp: ");
        Serial.print(Kp);
        Serial.print(", Ki: ");
        Serial.print(Ki);
        Serial.print(" Kd: ");
        Serial.println(Kd);
        
        actuator[n].setPIDGains(Kp, Ki, Kd);
        break;
// --------------------------------------------------------------------------------------------------------------------------------------------------
      case 'H':
        //Home Motor
        cmdValue = Serial.parseFloat();
        actuator[n].goHome(cmdValue);
        break;
// --------------------------------------------------------------------------------------------------------------------------------------------------
      case 'E':
        // Use end effector
        
        break;
// --------------------------------------------------------------------------------------------------------------------------------------------------
      default:
        // If command identifier is not recognized
        Serial.print("COMMAND IDENTIFIER ");
        Serial.print(cmd);
        Serial.println(" not recognized");
        break;
    }
}
