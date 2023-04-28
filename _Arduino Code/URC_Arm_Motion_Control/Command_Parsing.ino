/* Interpret command and determing which actions to take

  COMMAND FORMAT: nCv
  n - Joint Number (1,2,3,4,5,6)
  C - Command Identifier (char)
  v - Command Value (int) - MUST be 4 characters long (ex: -034 or 0264)
  
 */

// Positions of joints 5 and 6 for use in differential mechanism calculations
float J5_pos = 0;
float J6_pos = 0;

// Speed of joints 5 and 6 for use in differential mechanism calculations
float J5_speed = 0;
float J6_speed = 0;

void parseCommand(char cmd[]) {
    // Initialize command value
    float cmd_val = 0;

    // Convert joint number from character to integer (ASCII value of 0 is 48) then then subtract one to get array index
    int joint_num = cmd[0] - 49;
    // Extract command identifier
    char cmd_id = cmd[1];

    // Convert lower-case letters to upper-case by manupulating ASCII value of char
    // Capital letters always have ASCII value 32 lower than lower-case letters
    if (cmd_id > 90) {cmd_id -= 32;}

    // Extract command value, determine if value is negative, then convert to integer
    if (cmd[2] == '-') {
      cmd_val = (cmd[3] - 48)*100 + (cmd[4] - 48)*10 + (cmd[5] - 48);
      cmd_val *= -1;
    } else {
      cmd_val = (cmd[2] - 48)*1000 + (cmd[3] - 48)*100 + (cmd[4] - 48)*10 + (cmd[5] - 48);
    }
    
    switch (cmd_id) {
// --------------------------------------------------------------------------------------------------------------------------------------------------
      case 'P':  // Set new command position for automatic position control
        // Only use this command if automatic position control is active
        if (!automaticMode) {break;}

        // If setting position of joint 5 or 6, use equation for differential mechanism to calculate needed position of actuators 5 and 6
        // DiffA Act Pos = Joint6 + Joint5
        // DiffB Act Pos = Joint6 - Joint5
        if (joint_num == 4) {
          J5_pos = cmd_val;
          actuator[4].sendTargetPosition(J6_pos + J5_pos);
          actuator[5].sendTargetPosition(J6_pos - J5_pos);
        } else if (joint_num == 5) {
          J6_pos = cmd_val;
          actuator[4].sendTargetPosition(J6_pos + J5_pos);
          actuator[5].sendTargetPosition(J6_pos - J5_pos);
        } 
        
        // Otherwise, just tell actuator its new target position directly
        else {
          actuator[joint_num].sendTargetPosition(cmd_val);
        }
        
        Serial.print("POSITION: ");
        Serial.println(cmd_val);
        break;
// --------------------------------------------------------------------------------------------------------------------------------------------------
      case 'S':  // Set new desired speed for manual speed control
        // Only use this command if automatic position control is not active (ie manual speed control is being used)
        if (automaticMode) {break;}

        // If setting position of joint 5 or 6, use equation for differential mechanism to calculate needed position of actuators 5 and 6
        // DiffA Act Pos = Joint6 + Joint5
        // DiffB Act Pos = Joint6 - Joint5
        if (joint_num == 4) {
          J5_speed = cmd_val;
          actuator[4].sendTargetSpeed(J6_speed + J5_speed);
          actuator[5].sendTargetSpeed(J6_speed - J5_speed);
        } else if (joint_num == 5) {
          J6_speed = cmd_val;
          actuator[4].sendTargetSpeed(J6_speed + J5_speed);
          actuator[5].sendTargetSpeed(J6_speed - J5_speed);
        }
        
        else {actuator[joint_num].sendTargetSpeed(cmd_val);}

        Serial.print("SPEED: ");
        Serial.println(cmd_val);
        break;
// --------------------------------------------------------------------------------------------------------------------------------------------------
      case 'R':  // Reset specified actuator to clear error or immediately stop motion
        // If command value is non-zero, actuator forgets its position
        // Otherwise, store position of actuator
        if (cmd_val != 0) {
          // Reset actuator
          actuator[joint_num].reset();
          // Reset encoder position or reclaibrate absolute encoder
          switch (joint_num) {
            case 0:
              enc_J1.write(0);
              break;
            case 1:
              enc_J2.write(0);
              break;
            case 2:
              actuator[joint_num].calibratePos(analogRead(J3FeedbackPin));
              break;
            default:
              break;
          }
        }
        // Otherwise, reset and store current position
        else {actuator[joint_num].reset( actuator[joint_num].getCurrPos() );}
        break;
// --------------------------------------------------------------------------------------------------------------------------------------------------
      case 'M':  // Set new maximum safe speed of specified actuator
        actuator[joint_num].sendSafeMaxSpeed(cmd_val);
        
        Serial.print("SPEED LIMIT: ");
        Serial.println(cmd_val);
        break;
// --------------------------------------------------------------------------------------------------------------------------------------------------
      case 'Q':  // Print status of actuator
        Serial.print("ACTUATOR ");
        Serial.print(joint_num+1);
        Serial.print(": ");
        actuator[joint_num].print();
        break;
// --------------------------------------------------------------------------------------------------------------------------------------------------
      case 'K':  // Set gains for PID algorithms
        double Kp = (double)Serial.parseFloat();
        double Ki = (double)Serial.parseFloat();
        double Kd = (double)Serial.parseFloat();

        Serial.print(" Kp: ");
        Serial.print(Kp);
        Serial.print(", Ki: ");
        Serial.print(Ki);
        Serial.print(" Kd: ");
        Serial.println(Kd);
        
        actuator[joint_num].setPIDGains(Kp, Ki, Kd);
        break;
// --------------------------------------------------------------------------------------------------------------------------------------------------
      case 'H': //Home Motor
        actuator[joint_num].goHome(cmd_val);
        break;
// --------------------------------------------------------------------------------------------------------------------------------------------------
      case 'E':
        // Use end effector
        
        break;
// --------------------------------------------------------------------------------------------------------------------------------------------------
      default:
        // If command identifier is not recognized
        Serial.print("COMMAND IDENTIFIER ");
        Serial.print(cmd_id);
        Serial.println(" not recognized");
        break;
    }
}
