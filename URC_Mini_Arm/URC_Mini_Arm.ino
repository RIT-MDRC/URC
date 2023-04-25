/*  Controller for URC mini-arm
 *  MICROCONTROLLER: Teensy 4.1 - https://www.pjrc.com/store/teensy41.html
 */

// Input pins for reading potentiometers
int pot_J1 = 14;
int pot_J2 = 15;
int pot_J3 = 16;
int pot_J4 = 17;
int pot_J5 = 18;
int pot_J6 = 19;
int pot_G  = 20;
// Store in array for easy access
int potPin[7] = {pot_J1, pot_J2, pot_J3, pot_J4, pot_J5, pot_J6, pot_G};

int joint_values[7] = {};

// Input pin for activation switch
int activationSwitch = 37;

// Is data being sent to arm?
bool sending = false;

//TIME VARIABLES
int timer;   // Holds time since last sending command to motor controller
int TIME;         // Holds last recorded time in milliseconds
int t = 0;
 
void setup() {
  Serial.begin(115200);
  
  // Initialize analog input pins
  for (int n = 0; n < 7; n++) {pinMode(potPin[n],INPUT);}

  // Initialize activation switch and read its state
  pinMode(activationSwitch,INPUT);
  sending = digitalRead(activationSwitch);

  // Initialize time variables
  timer = 0;
  TIME = millis();
}

void loop() {
  // Read the current time 
  int newTIME = millis();
  // Calculate change in time since last reading and keep track with timer
  timer += newTIME - TIME;
  // Store current time in TIME variable to use next loop
  TIME = newTIME;

  if (timer > 50) {
    for (int n = 0; n < 7; n++) {joint_values[n] = analogRead(potPin[n]);}
    
    Serial.println(joint_values[0]);
 
    sending = digitalRead(activationSwitch);
    if (sending) {
      
    }
    
    timer = 0;
  }
}
