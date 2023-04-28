// FUNCTIONS
// Read from gripper select switch and determine which way gripper should be going
void readGripperSwitch() {
  // Read switch state
  int val = analogRead(gripper_pin);

  // Store last loop's reading
  last_dir = gripper_dir;
  
  if (val > 600) {
    // CLOSING
    gripper_dir = 1;
  } else if (val < 400) {
    // OPENING
    gripper_dir = -1;
  } else {
    // HOLDING
    gripper_dir = 0;
  }

  // Only send a command when the switch state changes
  if ((gripper_dir != 0 && last_dir == 0) || (gripper_dir == 0 && last_dir != 0)) {
    Serial2.println(gripperGCODE());
  }
}

String gripperGCODE() {
  // Initialize command format (second char will always be 'P' for setting position)
  char gcode[7] = "0E0000";
  
  // Create local variable for determing what command value should be
  int dir = gripper_dir;

  // If direction is negative, add negative sign to command and take abs value of direction value
  if (dir < 0) {
    gcode[2] = '-';
    dir *= -1;
  }

  // If direction is not zero, then put a one in the command (actual value doesn't matter)
  if (dir != 0) {gcode[3] = '1';}

  return String(gcode);
}
