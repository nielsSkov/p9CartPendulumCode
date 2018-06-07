/*
    Main for swing-up and stabilization of inverted pendulum 
    Created by Jesper Hede Christensen and Rasmus Christiansen, Spring 2017

    Commands
    'L'       Enables or disables the serial log
    'r'       Resets sensors and Extended Kalman Filter
    '1'       Enables the control 
    '0'       Disables the control
    'swup=x'  Chooses swing-up method
    'stab=y'  Chooses stabilization method

    Swing-up methods
    1)        Full system energy 
    2)        Pendulum only energy    (chosen as standard, best performance)
    3)        Aastroem sign based 

    Stabilization methods
    1)        PID
    2)        LQG
    3)        Sliding-mode            (chosen as standard)

    Initialization process:
    Place the cart at the left-hand side position with the pendulum hanging down, then press 'r'
    (Might be necessary to utilize the John-hammer to make sure the pendulum is in fact hanging straight down)

    Serial settings:
    * Baudrate    115200
    * Newline terminator
    * Serial output:  time, x, th, xd, thd, u, swupmode, stabmode, looptime

    NB: Logging is disabled during control action to ensure a looptime less than 5 ms. The code was originally implemented on the native USB port which has a faster serial bus.
*/

#include <JointGR171030.h>
#include <ctrlGR171030.h>
#include <EKFGR171030.h>

// Definitions
#define SAMPLINGTIME 0.005
#define ENABLEPENDUL 50
#define ENABLESLED 48

// Create joint objects
Joint sled(1, SAMPLINGTIME);
Joint pendu(2, SAMPLINGTIME);

// Setup system
void setup() {
  // HACK
  // Pin 34 needs to be input because of external clock (Remove when corrected)
  pinMode(34, INPUT);

  ////////////////////////
  // Setup Serial
  ///////////////////////
  Serial.begin(115200);

  while (!Serial) {
    // Wait for serial to establish connection
  }
  Serial.flush();

  ////////////////////////
  // Setup pin modes
  ////////////////////////
  // Data bus
  pinMode(40, INPUT);
  pinMode(41, INPUT);
  pinMode(42, INPUT);
  pinMode(43, INPUT);
  pinMode(44, INPUT);
  pinMode(45, INPUT);
  pinMode(46, INPUT);
  pinMode(47, INPUT);

  ///////////////////////
  // Setup pendulum system
  //////////////////////
  // Initialize joints
  sled.init();
  pendu.init();

  // Reset positions
  sled.resetPos();
  pendu.resetPos();

  // Set DAC+ADC resolution to 12 bit
  analogWriteResolution(12);
  analogReadResolution(12);

  // Set output to zero
  sled.setOutput(0);
  pendu.setOutput(0);

  // setup Enable motor driver outputs (FOR SAFTY REASONS, DO NOT CHANGE THE BELOW 6 LINES)
  digitalWrite(ENABLEPENDUL, LOW);
  digitalWrite(ENABLESLED, LOW);
  pinMode(ENABLEPENDUL, OUTPUT);
  pinMode(ENABLESLED, OUTPUT);
  digitalWrite(ENABLEPENDUL, LOW);
  digitalWrite(ENABLESLED, LOW);

  // Wait for a character on the serial line
  establishContact();
}

// Create kalman object
EKF kalman(SAMPLINGTIME, sled.readPos(), pendu.readPos());

// ------------- GLOBAL VARIABLES ---------- //

// State related values
float posSled = 0;
float posPend = 0;
float states[5] = {0,0,0,0,0}; // Including debug output

// Control related variables
float ctrlOut[2] = {0,0};     // Including debug output
int swupmode = 2;
int stabmode = 3;
float setOutSled = 0;
float setOutAct = 0;
float xRef = 0.38; 

// Time related variables
unsigned long timestart = 0;
unsigned long timeend = 0;
unsigned long timeact = 0;
int loopTimeViolated = 0;

// Serial comms related variables
byte msg = 0;
String inputString;
char serialOutput[200];
boolean logOn = false;

void loop() {

  // Start time counter 
  timestart = micros();

  // Read user input
  readSerial();

  ////////////////////////////////////////////
  // Read positions and get state estimate
  ////////////////////////////////////////////
  posSled = sled.readPos();   // Sled
  posPend = pendu.readPos();  // Pendulum

  kalman.getStateEstimates(states, posSled, posPend, setOutSled);

  ////////////////////////////////////////////
  // Main structure - switch on input message
  ////////////////////////////////////////////
  switch (msg) {
    // Stop Motors
    case '0':
      digitalWrite(ENABLEPENDUL, LOW);
      digitalWrite(ENABLESLED, LOW);
      setOutSled = 0;
      break;

    // Start control
    case '1':
      digitalWrite(ENABLEPENDUL, HIGH);
      digitalWrite(ENABLESLED, HIGH);
      ctrl(ctrlOut, xRef, states, swupmode, stabmode);
      setOutSled = ctrlOut[0];

      // Stop log to ensure performance during control action
      if (logOn)
        Serial.println("Stopping log to ensure performance.");
        logOn = false;
      break;

    // Reset sensors, EKF and control variables
    case 'r':
      kalman.resetEKF(true);
      resetCtrl(true);
      pendu.resetPos();
      sled.resetPos();
      msg = '0';
      break;

    // Turn on/off log
    case 'L':
      if (!logOn)
        logOn = true;
      else
        logOn = false;

      msg = '0';   // Ensures it does not continue to switch between ON/OFF
      break;
  }

  ////////////////////////////////////////////
  // Apply output
  ////////////////////////////////////////////
  setOutAct = sled.setOutput(setOutSled);
  pendu.setOutput(0);

  ////////////////////////////////////////////
  // Write log
  ////////////////////////////////////////////
  if (logOn) {
    createLog();
    Serial.println(serialOutput);
  }

  /////////////////////////////////////////////
  // Looptime control
  /////////////////////////////////////////////
  timeend = micros();
  timeact = timeend - timestart;

  if (timeact < 5000) {
    delayMicroseconds(5000 - timeact);
    loopTimeViolated = 0;
  }
  else {
    loopTimeViolated = 1;
  }
}

// Create string to be sent on serial line
void createLog() {
    /* Variable       Identifier
     *  time          %d
     *  states[0-3]   %+f
     *  u             %+f
     *  stab          %d
     *  swup          %d
     *  looptime      %lu
    */
    sprintf(serialOutput, "%d, %+f, %+f, %+f, %+f, %+f, %d, %d, %lu", timestart/1000, states[0], states[1], states[2], states[3], setOutSled, swupmode, stabmode, timeact);
}

// Read serial input until '\n'
void readSerial() {
  if (Serial.available() != 0) {
    inputString = Serial.readStringUntil('\n');

    if (inputString.length() > 1) {
      // If inputstring is larger than 1, we search for "=", and the charater afterwards
      int charPos = inputString.indexOf('=');
      char mode = inputString.charAt(charPos + 1);
      // Then, determine if it is swup or stab mode. Convert mode from char to int
      if (inputString.indexOf('w') != -1) {
        swupmode = mode - '0';
      }
      else {
        stabmode = mode - '0';
      }
    }
    else {
      msg = inputString.charAt(0);
    }
  }
}

// Awaits an character on the serial line before going into main loop
void establishContact() {
  while (Serial.available() <= 0) {
    Serial.println("Arduino Ready. Press 'L'");
    sled.setOutput(0);
    pendu.setOutput(0);
    delay(1000);
  }
}
