/* This code was developed by Kenville Haynes

Electrical and Electronic Engineering department 

University of Nottingham */

// Include header files
#include <Wire.h>
#include <Keypad.h>
#include <MPU6050_tockn.h>
#include <LiquidCrystal_PCF8574.h>

int mid_servo = 53;                                                           // Centre front steering
int leftMotor_speed = -205, rightMotor_speed = -150, servoAngle = mid_servo;  // Left motor speed, right motor speed and servo angle for EEEBot to travel straight
long enc1_count = 0, enc2_count = 0;                                          // Left and right encoder count
const float wheel_diameter = 6;                                               // Rear wheel diameter in cm
const float encoder_resolution = 24;                                          // Encoder counts per revolution
float target_angle = 35;                                                      // Target angle for left and right turns
float current_angle = 0;                                                      // Current turning angle
const float turn_step_angle = 5;                                              // Angle to turn in each step, in degrees

// LCD I2C address
LiquidCrystal_PCF8574 lcd(0x27);


// Setup MPU6050 sensor
MPU6050 mpu6050(Wire);

void setupMPU6050() {
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Serial.println("MPU6050 initialized");
}

// Commands array
#define MAX_COMMANDS 10
int commands[MAX_COMMANDS][2];
int commandCount = 0;

int selectedCommand;  // To store the command being entered

bool stopExecution = false;

// Keypad configuration
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};

byte rowPins[ROWS] = { 12, 14, 27, 26 };
byte colPins[COLS] = { 25, 33, 32, 15 };
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// Menu options
enum MenuState {
  MAIN_MENU,
  COMMAND_LIST,
  ADD_COMMAND,
  MOVE_COMMAND,
  TURN_COMMAND,
  ROBOT_STATUS,
  ENTER_DISTANCE
};

// Ask user for distance function
void printEnterDistance() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Enter distance");
  lcd.setCursor(0, 1);
  lcd.print("in 10cm: ");
}

// To update what's displayed on the LCD
void updateLcd() {
  lcd.clear();
  printCommands();
}

MenuState currentState = MAIN_MENU;  // Initialise the LCD to start at the main menu option

// Function to print the main menu to the LCD
void printMainMenu() {
  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print("Main Menu");
  for (int i = 0; i < 3; i++) {
    lcd.setCursor(0, 1);
    lcd.print("                ");  // Clear the option row
    if (i == 0) {
      lcd.setCursor(0, 1);
      lcd.print("1: Commands");
    } else if (i == 1) {
      lcd.setCursor(0, 1);
      lcd.print("2: Robot Status");
    } else if (i == 2) {
      lcd.setCursor(0, 1);
      lcd.print("3: Go");
    }
    delay(1000);  // Display each option for 1s
  }
  // Return to the main menu
  lcd.setCursor(4, 0);
  lcd.print("Main Menu");
  lcd.setCursor(0, 1);
  lcd.print("1:Cmd");
  lcd.setCursor(6, 1);
  lcd.print("2:Sts");
  lcd.setCursor(12, 1);
  lcd.print("3:Go");
}

// Function to print the list of commands entered
void printCommandList() {
  if (commandCount == 0) {

    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Commands Menu");

    lcd.setCursor(0, 1);  // Set the row to 2 to display the command options below the command list
    lcd.print("*:Bk #:Add D:Clr");
  }

  printCommands();
}


// Function to ask the user what command they want to enter
void printAddCommandMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("1:Move  2:Turn");
  lcd.setCursor(0, 1);
  lcd.print("3:Back");
}


// Function to ask the user if they want to add a forward or backward movement
void printMoveCommandMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("1:Fwd  2:Bwd");
  lcd.setCursor(0, 1);
  lcd.print("3:Back");
}

// Return to the command menu
void printTurnCommandMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("1:Left  2:Right");
  lcd.setCursor(0, 1);
  lcd.print("3:Back");
}

// EEEBot status function
void printRobotStatus() {

  String message = "Maze Naigation mode selected.";
  int messageLength = message.length();
  int displayLength = 16;  // adjust this to match the width of your display
  int scrollOffset = 0;

  while (scrollOffset <= messageLength - displayLength) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(message.substring(scrollOffset, scrollOffset + displayLength));
    lcd.setCursor(0, 1);  // Move cursor to the second row
    lcd.print("*:Back");  // Print the "*:Back" message
    delay(500);           // Adjust the delay to control the scrolling speed
    scrollOffset++;
  }

  // Once the message has been fully scrolled, display without scrolling
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(message.substring(scrollOffset));
  lcd.setCursor(0, 1);  // move cursor to the second row
  lcd.print("*:Back");  // print the "*:Back" message
}

// Update menu function
void updateMenu() {
  switch (currentState) {
    case MAIN_MENU:
      printMainMenu();
      break;
    case COMMAND_LIST:
      printCommandList();
      break;
    case ADD_COMMAND:
      printAddCommandMenu();
      break;
    case MOVE_COMMAND:
      printMoveCommandMenu();
      break;
    case TURN_COMMAND:
      printTurnCommandMenu();
      break;
    case ROBOT_STATUS:
      printRobotStatus();
      break;
    case ENTER_DISTANCE:
      printEnterDistance();
      break;
  }
}

//Setup function
void setup() {

  Wire.begin();

  lcd.begin(16, 2);  // Initialize a 16x2 LCD
  lcd.setBacklight(255);
  lcd.home();
  lcd.clear();

  setupMPU6050();

  // Initialize the menu
  updateMenu();
}

// Adding the commands entered to the list
void addCommand(int movement, int distance = 0) {
  if (commandCount < MAX_COMMANDS) {
    if (movement == 0 && commandCount > 0 && commands[commandCount - 1][0] == 0) {
      // If the current command is a forward movement and the previous command is also a forward movement, combine them
      commands[commandCount - 1][1] += distance;
    } else {
      commands[commandCount][0] = movement;
      commands[commandCount][1] = distance;
      commandCount++;
    }
    updateLcd();
  }
}

// Print the commands entered
void printCommands() {
  int cmdPerRow = 5;
  int maxRows = 2;
  for (int i = 0; i < commandCount; i++) {
    lcd.setCursor((i % cmdPerRow) * (16 / cmdPerRow), i / cmdPerRow);
    if (commands[i][0] == 1) {
      lcd.print("F");
    } else if (commands[i][0] == 2) {
      lcd.print("B");
    } else if (commands[i][0] == 3) {
      lcd.print("L");
    } else if (commands[i][0] == 4) {
      lcd.print("R");
    }
    lcd.print(commands[i][1]);
    if (i < commandCount - 1 && (i + 1) % cmdPerRow != 0) {  // Add this condition to avoid printing a comma after the last command
      lcd.print(" ");
    }
  }
}

// Clear commands function
void clearCommands() {
  for (int i = 0; i < MAX_COMMANDS; i++) {
    commands[i][0] = -1;
    commands[i][1] = 0;
  }
  commandCount = 0;
  updateLcd();
}

// Execute commands function
void executeCommands() {
  for (int i = 0; i < commandCount && !stopExecution; i++) {  // Add the stopExecution check
    int movement = commands[i][0];
    int distance = commands[i][1];
    executeMovement(movement, distance);
  }
  stopExecution = false;  // Reset the stopExecution flag after all commands are executed
}

// Transmit motor speed and servo position to Arduino Nano over I2C
void Transmit_to_arduino(int leftMotor_speed, int rightMotor_speed, int servoAngle) {
  const int I2C_SLAVE_ADDR = 4;  // Assuming the Nano has an I2C address of 4

  Wire.beginTransmission(I2C_SLAVE_ADDR);

  Wire.write((byte)((leftMotor_speed & 0x0000FF00) >> 8));
  Wire.write((byte)(leftMotor_speed & 0x000000FF));

  Wire.write((byte)((rightMotor_speed & 0x0000FF00) >> 8));
  Wire.write((byte)(rightMotor_speed & 0x000000FF));

  Wire.write((byte)((servoAngle & 0x0000FF00) >> 8));
  Wire.write((byte)(servoAngle & 0x000000FF));

  Wire.endTransmission();
}

// Request encoder count over I2C from the Arduino Nano
void requestEncoderValues(long &enc1_count, long &enc2_count) {
  const byte ARDUINO_NANO_ADDRESS = 4;  // The I2C address of the Arduino Nano
  const byte NUM_BYTES = 8;             // The number of bytes we are expecting (4 bytes per encoder)

  Wire.requestFrom(ARDUINO_NANO_ADDRESS, NUM_BYTES);

  if (Wire.available() == NUM_BYTES) {
    enc1_count = Wire.read() << 24 | Wire.read() << 16 | Wire.read() << 8 | Wire.read();
    enc2_count = Wire.read() << 24 | Wire.read() << 16 | Wire.read() << 8 | Wire.read();
  }
}

// Calculate distance travelled in cm by the EEEBot
float calculateDistance(long encoder_count, float wheel_diameter, float encoder_resolution) {
  float wheel_circumference = wheel_diameter * PI;
  float distance = (wheel_circumference / encoder_resolution) * encoder_count;
  return distance;
}

// Execute the entered commands
void executeMovement(int movement, int distance) {

  int calibrated_value = 0.7;  // Distance travelled calibrated value to stop the EEEBot to a more accurately

  requestEncoderValues(enc1_count, enc2_count);
  float traveled_distance = 0;
  float target_angle = 35;  // Target angle for turning (90 degrees)

  if (movement == 1) {  // Forward

    long initial_enc1_count, initial_enc2_count;
    requestEncoderValues(initial_enc1_count, initial_enc2_count);  // Store initial encoder counts

    leftMotor_speed = -205;
    rightMotor_speed = -150;
    servoAngle = 53;

    // Continue moving until the desired incremental distance is reached
    while (traveled_distance < (distance * calibrated_value)) {
      Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
      //delay(100);
      long current_enc1_count, current_enc2_count;
      requestEncoderValues(current_enc1_count, current_enc2_count);

      // Calculate incremental distance traveled from the initial encoder counts
      long incremental_enc1_count = current_enc1_count - initial_enc1_count;
      traveled_distance = calculateDistance(incremental_enc1_count, wheel_diameter, encoder_resolution);
      traveled_distance = abs(traveled_distance);
    }

    leftMotor_speed = 0;
    rightMotor_speed = 0;
    servoAngle = 53;
    Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);


  } else if (movement == 2) {  // Backward
    long initial_enc1_count, initial_enc2_count;
    requestEncoderValues(initial_enc1_count, initial_enc2_count);  // Store initial encoder counts

    leftMotor_speed = 205;
    rightMotor_speed = 150;
    servoAngle = 53;

    // Continue moving until the desired incremental distance is reached
    while (traveled_distance < (distance * calibrated_value)) {
      Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
      //delay(100);
      long current_enc1_count, current_enc2_count;
      requestEncoderValues(current_enc1_count, current_enc2_count);

      // Calculate incremental distance traveled from the initial encoder counts
      long incremental_enc1_count = current_enc1_count - initial_enc1_count;
      traveled_distance = calculateDistance(incremental_enc1_count, wheel_diameter, encoder_resolution);
      traveled_distance = abs(traveled_distance);
    }

    leftMotor_speed = 0;
    rightMotor_speed = 0;
    servoAngle = 53;
    Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);

  } else if (movement == 3) {  // Turn left

    float current_angle = 0;  // Reset the current angle before turning
    leftMotor_speed = -205;
    rightMotor_speed = -150;
    servoAngle = 100;

    while (abs(current_angle) < target_angle) {
      float step_angle = 0;
      while (abs(step_angle) < turn_step_angle) {
        Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
        delay(10);  // Time step for angle calculation
        mpu6050.update();
        float delta_angle = mpu6050.getGyroZ() * (10.0 / 1000.0);
        current_angle += delta_angle;
        step_angle += delta_angle;
      }

      Transmit_to_arduino(0, 0, 53);
      delay(250);
      Transmit_to_arduino(205, 150, 0);
      delay(250);
    }

  } else if (movement == 4) {  //Turn right

    float current_angle = 0;  // Reset the current angle before turning
    leftMotor_speed = -205;
    rightMotor_speed = -150;
    servoAngle = 0;

    while (abs(current_angle) < target_angle) {
      float step_angle = 0;
      while (abs(step_angle) < turn_step_angle) {
        Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
        delay(10);  // Time step for angle calculation
        mpu6050.update();
        float delta_angle = mpu6050.getGyroZ() * (10.0 / 1000.0);
        current_angle += delta_angle;
        step_angle += delta_angle;
      }
      Transmit_to_arduino(0, 0, 53);
      delay(250);
      Transmit_to_arduino(205, 150, 100);
      delay(250);
    }
  }
  leftMotor_speed = 0;
  rightMotor_speed = 0;
  servoAngle = 53;
  Transmit_to_arduino(leftMotor_speed, rightMotor_speed, servoAngle);
  delay(500);
}

// Run all the commands entered function
void runAllCommands() {
  lcd.clear();  // Clear the LCD before displaying the countdown

  for (int i = 5; i > 0; i--) {
    lcd.setCursor(0, 0);
    lcd.print("Starting in: ");
    lcd.print(i);
    delay(1000);  // Wait for 1 second
  }

  lcd.clear();  // Clear the LCD after the countdown is finished

  // Execute all the commands in the array
  for (int i = 0; i < commandCount; i++) {
    int movement = commands[i][0];
    int distance = commands[i][1];
    executeMovement(movement, distance);
  }

  // Clear the commands array and update the menu to show the main menu
  clearCommands();
  currentState = MAIN_MENU;
  updateMenu();
}

// Process the input from the 4x4 keypad
void processMenuInput(char key) {
  switch (currentState) {
    case MAIN_MENU:
      if (key == '1') currentState = COMMAND_LIST;
      else if (key == '2') currentState = ROBOT_STATUS;
      else if (key == '3') runAllCommands();  // add this line
      break;
    case COMMAND_LIST:
      if (key == '*') currentState = MAIN_MENU;
      else if (key == '#') currentState = ADD_COMMAND;
      else if (key == 'D') {
        clearCommands();
        currentState = COMMAND_LIST;
      }
      break;

    case ADD_COMMAND:
      if (key == '1') currentState = MOVE_COMMAND;
      else if (key == '2') currentState = TURN_COMMAND;
      else if (key == '3') currentState = COMMAND_LIST;
      break;
    case MOVE_COMMAND:
      if (key == '1') {
        selectedCommand = 1;
        currentState = ENTER_DISTANCE;
      } else if (key == '2') {
        selectedCommand = 2;
        currentState = ENTER_DISTANCE;
      } else if (key == '3') currentState = ADD_COMMAND;
      break;
    case TURN_COMMAND:
      if (key == '1') {
        addCommand(3);
        currentState = COMMAND_LIST;
      } else if (key == '2') {
        addCommand(4);
        currentState = COMMAND_LIST;
      } else if (key == '3') currentState = ADD_COMMAND;
      break;
    case ROBOT_STATUS:
      if (key == '*') currentState = MAIN_MENU;
      break;

    case ENTER_DISTANCE:
      if (key >= '1' && key <= '9') {
        int distance = (key - '0') * 10;
        addCommand(selectedCommand, distance);
        currentState = COMMAND_LIST;
      } else if (key == '*') {
        currentState = MOVE_COMMAND;
      }
      break;
  }
  updateMenu();
}

// Main loop
void loop() {
  char key = keypad.getKey();

  if (key) {
    processMenuInput(key);
  }
}
