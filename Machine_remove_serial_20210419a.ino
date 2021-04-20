// [move_25$fold$move_39$fold$move_120$fold$move_39$fold$move_235$fold$move_39$fold$move_120$fold$move_39$fold$move_215$shear]
// Moved position of check for "shear" operation to bottom of if-else pool because dropthrough never got to "sheardown" and "shearup"
#include <digitalIO-examples.h>

#include "ClearCore.h"

// Define all relays
#define ExecutingOperations IO0
#define pump IO0
#define shearRelayUp IO2
#define shearRelayDown IO3
#define foldRelay IO4

// Define all sensors
#define foldSensorUp A12
#define foldSensorDown A11
#define shearSensorUp A9
#define shearSensorDown A10
#define handSensor ?

// Define buttons
#define ESTOP DI6
#define pauseButton DI7

// Define motor
#define motor ConnectorM0

// Select the baud rate to match the target serial device
#define baudRate 115200

// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last time LED was updated
unsigned long shearPrevious = 0;

// Define the velocity and acceleration limits to be used for each move
const int velocityLimit = 1900000; // pulses per sec
const int accelerationLimit = 20000000; // pulses per sec^2

// constants won't change:
const long interval = 3000;     
const long shearTimeout = 500;

bool foldTrigger = false;
bool shearTrigger = false;
bool shearActive = false;

// Serial reading
//String serialBuffer[64];
int bufferPosition;
String commandBuffer;
bool awaitingSerial = false;
const char pauseChar[] = "pause";
const char startChar[] = "start";

bool newCommand = false;
bool expectingDistance = false;
bool ableToPrint = false;
bool pauseActive = false;

// Variables used for setting piece distance lengths
// through serial
double pieceLengths[8];
int lastPiece = 0;

int foldDelay = 0;

void setup() {
  pinMode(pump, OUTPUT);
  pinMode(foldRelay, OUTPUT);
  pinMode(shearRelayUp, OUTPUT);
  pinMode(shearRelayDown, OUTPUT);
  pinMode(ESTOP, INPUT);

  // Sets up serial communication and waits up to 5 seconds for a port to open.
  // Serial communication is not required for this example to run.
  Serial.begin(baudRate);
  uint32_t timeout = 5000;
  uint32_t startTime = millis();
  while (!Serial && millis() - startTime < timeout) {
    continue;
  }

  Serial1.begin(baudRate);
  Serial1.ttl(false);

  // MOTOR SETUP

  // Sets the input clocking rate. This normal rate is ideal for ClearPath
  // step and direction applications.
  MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);

  // Sets all motor connectors into step and direction mode.
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);
  
  // Set the motor's HLFB mode to bipolar PWM
  motor.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
  // Set the HFLB carrier frequency to 482 Hz
  motor.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
  
  // Sets the maximum velocity for each move
  motor.VelMax(velocityLimit);

  // Set the maximum acceleration for each move
  motor.AccelMax(accelerationLimit);

  motor.EnableRequest(true);

// Commented out 04/09/21 KCL.  I think that some modes of HLFB output start low until the first motion is complete
//  Serial.println("Waiting for HLFB.");
//  while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
//    continue;
//  }
//  Serial.println("Motor Ready");

}

void loop() {
  // put your main code here, to run repeatedly:

  bool printError = true;
  if (!checkESTOPStatus()) {
      readSerial();
      printError = false;
    } else {
      if (printError) {
        Serial.println("ERROR: ESTOP PRESSED");
        printError = false; 
      }
    }
    
}

void checkForPause() {

  bool startRead = false;
  int i = 0;
  while (Serial.available() > 0) {
    char serial = Serial.read();
     if (serial == '[') {
      startRead = true;
     } else if (serial == ']') {
      pauseActive = true;
      Serial.println("ERROR: Pause is now active");
      break;
     } else if (startRead) {
      
      if (serial == pauseChar[i]) {
        i++;
        continue;
      } else {
        Serial.println("ERROR: Attempting to send command while paused");
        break;
      }
      
     }
  }
}

void checkForUnPause() {

  bool startRead = false;
  int i = 0;
  while (Serial.available() > 0) {
    char serial = Serial.read();
     if (serial == '[') {
      startRead = true;
     } else if (serial == ']') {
      pauseActive = false;
      Serial.println("INFO: Pause is no longer active");
      break;
     } else if (startRead) {
      
      if (serial == startChar[i]) {
        i++;
        continue;
      } else {
        Serial.println("ERROR: Attempting to send command while paused");
        break;
      }
      
     }
  }
}

void readSerial() {

  // Serial will be read as it comes in
  // It will first look for the start of a command with
  // an opening bracket ([) to indicate the start of a command.
  // Then, it will split the commands in a string array seperated by the
  // dollar sign character ($). It will continue reading until a close 
  // bracket is found (]) then it will call the parseSerial() function
  // with the array of commands and the amount of commands to parse. 
  // REMOVED, Ignore:
  // In situations where serial may be delayed so a full command is not
  // sent in time, it will continue to wait until a close braket is found
  // until parsing the serial command. 
  // 
  // I've removed the code that waits for more serial before attempting to parse
  // as there are some random bugs that I need to fix, it also is unlikely that
  // a full serial command cannot be read at one time. In the situation it can't,
  // just notify the computer that the last command was not run. The program also
  // knows when it was not able to fully write to the port 

    // Holding a string array for the commands
    String stringBuffer[64];
    // A string variable that will concat the incoming
    // characters until a dollar sign is found
    String currentCommand = "";
    // Used to indicate which command we are on to put in the
    // stringBuffer array
    int command = 0;
    // Used to indicate if we have recieved serial or not
    bool parsedSerial = false;

    /*
    // This means that a full command was not sent last time we
    // recieved serial
    if (awaitingSerial) {

      // This will take the previous array of commands
      // and place them in the local serialBuffer array which
      // is sent to parseSerial();
      for (int i = 0; i < 8; i++) {
        stringBuffer[i] = serialBuffer[i];  
      }

      // Sets command to last command we left off at
      command = bufferPosition;
      // Sets the current command to where we left off
      currentCommand = commandBuffer;
      // We are no longer awaiting for more serial
      awaitingSerial = false;
    }
    */

    // Used to indicate later if we recieved serial or not
    bool foundSerial = false;
    while (Serial.available() > 0) {
      foundSerial = true;

      char serial = Serial.read();

      // Going through each character, determine which part
      // in the serial command we are. A close bracket is used
      // to indicate the full command has been sent and it can 
      // be parsed
      if (serial == ']') {
        // Placing the last command into the array
        stringBuffer[command] = currentCommand;
        // Used to indicate we are no longer looking for a new command
        newCommand = false;
        // Used to indicate that we have finished reading the full serial command
        parsedSerial = true;
        // Sends the array of commands as well as the number of commands to be parsed
        parseSerial(stringBuffer, command + 1);
        break;
      // We are expecting a new command
      } else if (newCommand) {
        // Dollar sign to indicate the start of a new command
        if (serial == '$') {
          // Place the currentCommand string into the string array since
          // we are no longer reading anymore characters for this command
          stringBuffer[command] = currentCommand;
          // Increases the amount of commands
          command++;
          // Resets the currentCommand to an empty string so it can be filled again
          currentCommand = "";
        } else {
          // We are expecting a new command, so concat this character to the
          // currentCommand string
          currentCommand.concat(serial);
        }
      // Used to indicate that a new command has been sent
      } else if (serial == '[') {

        // If we are already parsing a command and we recieve an opening
        // bracket beforte a close, a command was not sent right
        if (newCommand) {
          Serial.println("Invalid command");
        }

        // Since its an opening bracket, we should expect a command after this
        newCommand = true;
      }
    }

    // The full command was not sent, wait until more
    // serial is sent
    if (foundSerial && newCommand && !parsedSerial) {
      Serial.println("ERROR: Command did not close with a closing bracket");

      /*
      // Sets global variable commandBuffer to the currentCommand
      commandBuffer = currentCommand;

      // Copys the contents of stringBuffer to serialBuffer
      for (int i = 0; i < 8; i++) {
        serialBuffer[i] = stringBuffer[i];  
      }

      // Sets the global variable bufferPoisition to the local variable command
      bufferPosition = command;

      // We are expecting more serial
      awaitingSerial = true;
      */
    }
}

void parseSerial(String serialArray[], int commands) {

  // AVAILABLE COMMANDS:
  // move_x || feed_x - Moves the motor x amount of mm
  // fold - Calls the foldSensor() function
  // shear - Calls the shearSensor() function
  // setdist - Used to indicate that this command will contain all the piece lengths to be printed 
  // print - Calls the printPiece() function which uses the lengths supplied in setdist 
  // pause - Allows the computer to pause without having to press the pause button
  // start - If the program sent a pause, use the start command to start it again
  // MANUAL COMMANDS
  // sheardown - moves the shear position to down
  // shearup - moves the shear position to up
  // folddown - moves the fold bar position to down
  // foldup - moves the fold

  // Example command that would be sent and processed
  // Command sent: [fold$move_10$fold$move_10$shear]
  // Program will check for brackets to ensure we have
  // recieved the full command
  // Processed: fold$move_10$fold$move_10$shear
  //
  // Example command to set lengths of pieces 
  // [setdist$25$36$21$28$38$25$54$77$87]
  // Then you can print a piece using the lengths supplied above
  // [print]
  // You can print and set the distance in a single command as well, using
  // [print$25$39$120$39$235$39$120$39$200]
  // Combining print commands with other commands may not work, will need to test

  long current = millis();
  double lengths[8];
  bool printNow = false;
  digitalWrite(ExecutingOperations, true);  // set output low to indicate operations are in process  
  // Iterate through the serialArray with all of the commands
  for (int i = 0; i < commands; i++) {
    //Serial.println(serialArray[i]);

    // Pretty self-explanatory, compares the commandRun string with 
    // the available commands and runs whatever function it calls
    String commandRun = serialArray[i];
    if (commandRun == "fold") {
      foldSensor();
    } else if (commandRun == "shear") {
      shearSensor();
    } else if (commandRun.substring(0, 4) == "move" || commandRun.substring(0,4) == "feed") {
      // Gets the index of "_" to ensure that a distance has actually been sent with the command
      int index = commandRun.indexOf("_");
      if (index != -1) {
        // Gets the distance as a string and convert that to a double
        String distance = commandRun.substring(index + 1);
        moveMotor(distance.toDouble());
      // indexOf will return -1 if the character is not found
      } else {
        Serial.print("ERROR: Invalid move command @ ");
        Serial.println(commandRun);
      }
    } else if (commandRun == "sheardown") {
       shearDown();
    } else if (commandRun == "shearup") {
      shearUp(); 
    } else if (commandRun == "foldup") {
      foldUp();
    } else if (commandRun == "foldDown") {
      foldDown();
    } else if (commandRun == "foldtimed") {
      foldTimed(); 
    } else if (commandRun == "setdist") {
      // Used to indicate that the next characters will be numbers instead of commands
      expectingDistance = true;
    } else if (commandRun == "print") {
        // Checks if piece lengths have been set and then 

        // Checks if next command is number, if so then we know
        // to check for piece lengths and print them now
        if (commands == 10 && serialArray[i + 1].toDouble() != 0) {
          printNow = true;
        } else if (ableToPrint) {
          printPiece(pieceLengths);
        } else {
          Serial.println("ERROR: Attempted to print without setting piece lengths");
          //Serial.println("No piece lengths have been sent");
          //Serial.print("commands=");
          //Serial.println(commands);
        }
    } else if (commandRun.substring(0,6) == "delay=") {
      String distance  = commandRun.substring(6);
      foldDelay = distance.toDouble(); 
    } else if (commandRun == "pause") {
      pauseActive = true;
    } else if (commandRun == "start") {
      Serial.println("WARN: Attempted to start while already out of a paused state");
    }
    else {

      if (printNow) {
        lengths[i - 1] = commandRun.toDouble();
      } else if (expectingDistance) {
        pieceLengths[lastPiece] = commandRun.toDouble();
        lastPiece++;
      } else {
        Serial.print("ERROR: Invalid command=");
        Serial.println(commandRun); 
      }
    }

    checkForPause();
    while (digitalRead(pauseButton) || pauseActive) {
      checkForUnPause();
      continue;
    }
    
  }

  if (printNow) {

    if (lengths[8] != 0) {
      printPiece(lengths);  
    } else {
      Serial.println("ERROR: Not enough piece lengths sent in print statement");
    }
  }

  if (expectingDistance) {

    if (lastPiece < 8) {
      Serial.println("ERROR: Missing piece distance lengths, expecting 9");
    }
    
    expectingDistance = false;
    ableToPrint = true;
    lastPiece = 0;
  }
  digitalWrite(ExecutingOperations, false);  // set output low to indicate operations are complete  
  
  Serial.print("SUCCESS: Total operation took=");
  Serial.println(millis() - current);
}

void printPiece(double pieceLengths[]) {

  // Check senosrs to make sure everything is in the correct
  // position before continuing
  while (analogRead(foldSensorDown) > 2500 || analogRead(shearSensorDown) > 2500) {
    Serial.println("Waiting for foldbar and shear to be in correct position");
    digitalWrite(foldRelay, false);
    digitalWrite(shearRelayDown, false);
    digitalWrite(shearRelayUp, true);
  }
  digitalWrite(shearRelayUp, false);

  // TODO: determine how many inches need to be fed out
  // 0.0000432 in per pulse
  Serial.println("INFO: Starting new piece");
  long current = millis();
//  moveMotor(25);
  for (int i = 0; i < 9; i++) {
    // Check pause button
    checkForPause();
    while (digitalRead(pauseButton) || pauseActive) {
      checkForUnPause();
      continue;
    }

    //Serial.print("Executing piece length=");
    //Serial.println(pieceLengths[i]);
    
    // If this returns false, that means the motor
    // did not move. Before continuing we need to make
    // sure the motor actually moves
    if (!moveMotor(pieceLengths[i])) {
      // TODO: 
      // What should we do if the motor does not move?
      Serial.println("ERROR: Failed to move motor");
    }

    checkForPause();
    while (digitalRead(pauseButton) || pauseActive) {
      checkForUnPause();
      continue;
    }

    if (i != 8){
      foldSensor();
      delay(foldDelay);
    }

    //delay(750);
  }

  // Pause all operations while pause button is pressed
  checkForPause();
  while (digitalRead(pauseButton) || pauseActive) {
    checkForUnPause();
    continue;
  }
  
  shearSensor();
  Serial.print("INFO: Finished, total operation took");
  Serial.println(millis() - current);  
}

void foldSensor() {

  long current = millis();

  if (checkESTOPStatus()) {
    return;
  }

  // Wait until shear is in correct position before
  // moving fold bar
  long startMovement = millis();
  while(analogRead(shearSensorUp) < 2000) {
    digitalWrite(shearRelayUp, true);
    if (startMovement + 150 < millis()){
      Serial.println("WARN: shear timed out");
      break;
    }
    continue;
  }

  digitalWrite(shearRelayUp, false);

  if (checkESTOPStatus()) {
    return;
  }

  startMovement = millis();

  // Turn on the relay to cause the fold bar to come down
  digitalWrite(foldRelay, true);

  // Wait for the sensor to read that the fold bar is down
  while (analogRead(foldSensorDown) < 2500) {
    if (startMovement + 150 < millis()) {
      Serial.println("WARN: fold down timed out");
      break;
    }
    continue;
  }

  // Once the fold bar is down, turn off the relay to release the fold bar
  digitalWrite(foldRelay, false);

  startMovement = millis();

  // Wait for fold bar to return to regular state before continuing
  while (analogRead(foldSensorDown) > 2000) {
    if (startMovement + 100 < millis()) {
      Serial.println("WARN: fold up timed out");
      break;
    }
    continue;
  }

  Serial.print("INFO: Fold operation took ");
  Serial.println(millis() - current);

}

void foldTimed() {
  long startMovement = millis();
  digitalWrite(foldRelay, true);
  while (analogRead(foldSensorDown) < 2500) {
    continue;
  }

  /*
  while (startMovement + 10 > millis()) {
    continue;
  }
  */

  digitalWrite(foldRelay, false);
  Serial.print("foldTimed took=");
  Serial.println(millis() - startMovement);
}

void shearSensor() {

  long ShearRoutineStartMillis = millis();
  long MovementStarted = millis();
  if (checkESTOPStatus()) {
    return;
  }

  //Serial.print("sheerDown position=");
  //Serial.println(analogRead(shearSensorDown));

  digitalWrite(shearRelayUp, false);
  digitalWrite(shearRelayDown, true);
  long shearDownTime = millis();
  // Waits for the sensor to read that the shear is down
  while (analogRead(shearSensorDown) < 3000){
    if (MovementStarted + 185 < millis()) {
      Serial.println("WARN: shear down timed out");
      break; 
    }
    continue;
  }
  /*
  Serial.print("Waiting for shear to move down took=");
  Serial.println(millis() - shearDownTime);
  */

  // Wait until the hand sensors is reading HIGH
  // meaning that there is a hand in front of the sensor
  // Once there is a hand in front of the sensor, the 
  // program will continue to run and shear the last piece
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;
  while (distance > 100) {
    Serial.println("INFO: Waiting for hand sensor");
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2;
    continue;
  }
  
  digitalWrite(shearRelayDown, false);
  digitalWrite(shearRelayUp, true);
  MovementStarted = millis();
  // Waits until the sensor to read that the shear is up
  while (analogRead(shearSensorUp) < 3000) {
    if (MovementStarted + 155 < millis()){
      Serial.println("WARN: shear up timed out");
      break;
    }
    continue;
  }
  //Serial.print("Waiting for shear to move up took=");
  //Serial.println(millis() - MovementStarted);
  digitalWrite(shearRelayUp, false);
  Serial.print("INFO: Shear finished, took ");
  Serial.println(millis() - ShearRoutineStartMillis);
  
}

void shearTimed() {
  long current = millis();
  digitalWrite(shearRelayDown, true);
  while (analogRead(shearSensorDown) < 2500) {
    continue;
  }
  digitalWrite(shearRelayDown, false);
  digitalWrite(shearRelayUp, true);
  long startUp = millis();
  while (startUp + 50 > millis()) {
    continue;
  }
  digitalWrite(shearRelayUp, false);
  Serial.print("shear operation took=");
  Serial.println(millis() - current);
}

void shearDown() {
  digitalWrite(shearRelayUp, false);
  digitalWrite(shearRelayDown, true);
}

void shearUp() {
  digitalWrite(shearRelayDown, false);
  digitalWrite(shearRelayUp, true);
}

void foldUp() {
  digitalWrite(foldRelay, true);
}

void foldDown() {
  digitalWrite(foldRelay, false);
}

bool checkESTOPStatus() {

  if (digitalRead(ESTOP)) {
    return false;
  }

  digitalWrite(shearRelayDown, false);
  digitalWrite(shearRelayUp, false);
  digitalWrite(foldRelay, false);

  // Move everything back to correct starting position
  if (analogRead(shearSensorDown) > 2000) {
    digitalWrite(shearRelayUp, true);
    long shearStart = millis();
    while (analogRead(shearSensorDown) > 2000) {

      if (millis() - shearStart > 210) break;
      
      continue;
    }
    digitalWrite(shearRelayUp, false);
  }

  return true;
  
}

bool moveMotorInches(double inches) {
  return moveMotor(inches * 25.4);
}

bool moveMotor(double mm) {

  long current = millis();

  if (checkESTOPStatus()) {
    return false;
  }

  if (motor.StatusReg().bit.AlertsPresent) {
        Serial.println("Motor status: 'In Alert'. Move Canceled.");
        return false;
    }

    // Convert mm to inches to determine how far to move the motor
    double converted = mm / 25.4;
    double stepsTaken = converted / 0.0000432;

    if (stepsTaken > 0) stepsTaken = stepsTaken * -1;
    
    //Serial.print("Converting mm to inches= ");
    //Serial.println(converted);

    // Check senosrs to make sure everything is in the correct
  // position before continuing
  while (analogRead(foldSensorDown) > 3000 || analogRead(shearSensorDown) > 3000) {
    digitalWrite(foldRelay, false);
    digitalWrite(shearRelayDown, false);
    digitalWrite(shearRelayUp, true);
  }
  digitalWrite(shearRelayUp, false);

  // Check again if ESTOP is pressed since 
  // we're not sure how long it could take for 
  // everything to be in position
  if (checkESTOPStatus()) {
    return false;
  }

    //Serial.print("Moving distance: ");
    //Serial.println(stepsTaken);

    // Command the move of incremental distance
    motor.Move(stepsTaken);

    // Waits for HLFB to assert (signaling the move has successfully completed)
    //Serial.println("Moving. Waiting for HLFB");
    while (!motor.StepsComplete() || motor.HlfbState() != MotorDriver::HLFB_ASSERTED) {
        continue;
    }
    Serial.print("INFO: Motor move took=");
    Serial.println(millis() - current);

    return true;
}

// Time per rev (64,000)
// 175, 303, 432, 560, 688
// Time per inch
// 94, 162, 187, 232, 300, 347, 372, 439, 464, 530, 556, 603, 671, 696, 760
// Time per 1/10th of inch
// 72, 79, 85, 89, 93, 75, 81, 85, 90, 116
// 1 inch: 23148 (0.0000432 in per pulse)
// mm = inch * 25.4
void motorTest(int i) {
  long current = millis();
  moveMotor((-23148 / 10) * i);
  Serial.print("Motor move time (");
  Serial.print(i);
  Serial.print(") ");
  Serial.println(millis() - current);
}
