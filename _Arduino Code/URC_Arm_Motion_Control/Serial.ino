/* Read command from either Serial Terminal or Bluetooth UART
 * Command MUST be 6 characters long (not including terminating character)
 */

// BLUETOOTH ------------------------------------------------------------------------------------------------------------------------------------
// Char array for storing command as it is read
char blue_cmd[7] = "000000";
// Index of next char to be read
int blue_cmd_index = 0;

void parseBluetoothCommand() {
  // Read next character
  char next_char = Serial2.read();

  if (next_char != '\n') {
    // If char wasn't a terminating char, then store char and increment index
    blue_cmd[blue_cmd_index] = next_char;
    blue_cmd_index += 1;
  } else {
    // If it was a terminating char, then the command has been fully read
    // Interpret the command and reset the index
    parseCommand(blue_cmd);
    blue_cmd_index = 0;

    // Print the recieved command onto the Serial Terminal
    Serial.println(blue_cmd);
  }
}

// SERIAL TERMINAL -------------------------------------------------------------------------------------------------------------------------------
// Char array for storing command as it is read
char term_cmd[7] = "000000";
// Index of next char to be read
int term_cmd_index = 0;

void parseTerminalCommand() {
  // Read next character
  char next_char = Serial.read();

  if (next_char != '\n') {
    // If char wasn't a terminating char, then store char and increment index
    term_cmd[term_cmd_index] = next_char;
    term_cmd_index += 1;
  } else {
    // If it was a terminating char, then the command has been fully read
    // Interpret the command and reset the index
    parseCommand(term_cmd);
    term_cmd_index = 0;

    // Print the recieved command onto the Serial Terminal
    //Serial.println(term_cmd);
  }
}
