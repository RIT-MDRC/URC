/* Read input from Serial Terminal and interpret it

  COMMAND FORMAT: Cn v
  C - Command Identifier (char)
  n - Joint Number (1,2,3,4,5,6)
  v - Command Value (int)
  
 */
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
      case 'P':
        // Set new command position for position algorithm
        cmdValue = Serial.parseFloat();
        joints[n].setNewPosition(cmdValue);
        
        Serial.print("POSITION: ");
        Serial.println(cmdValue);
        break;
      case 'S':
        // Set new max speed for joint
        cmdValue = Serial.parseFloat();
        if (joints[n].setMaxSpeed(cmdValue)) {        
          /*
          Serial.print("SPEED: ");
          Serial.println(setSpeed);
          */
        }
        break;
      case 'R':
        // Reset motor driver and set position and speed to zero
        joints[n].reset();

        switch (n) {
          case 0:
            enc_J1.write(0);
            break;
          case 1:
            enc_J2.write(0);
            break;
          case 2:
            joints[n].reset(analogRead(potPin));
            break;
        }
        //Serial.print("RESET ");
        //Serial.println(n+1);
        break;
      case 'Q':
        // Print status of motor
        Serial.print("JOINT ");
        Serial.print(n+1);
        Serial.print(": ");
        joints[n].print();
        //Serial.print(String(timer)+"\n");
        break;
      case 'D':
        //Drive Motor speed directly
        cmdValue = Serial.parseFloat();
        joints[n].driveMotor(cmdValue);
        break;
      case 'H':
        //Home Motor
        cmdValue = Serial.parseFloat();
        joints[n].homing(cmdValue);
        break;
      case 'G':
        // Actuate Gripper
        actuateGripper(n+1);
        break;
      default:
        break;
    }
}
