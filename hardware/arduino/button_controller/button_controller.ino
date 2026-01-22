/*
 * Week 7: Arduino Button Controller
 *
 * This Arduino sketch reads button inputs and sends UART commands
 * to control the SO-101 robot arm.
 *
 * Hardware Setup:
 * - 12 buttons (2 per joint: positive and negative direction)
 * - Optional: 1 emergency stop button
 * - Optional: Status LED
 *
 * Wiring (example for Arduino Nano):
 *   Button J1+ → Pin 2  (with pull-up, active LOW)
 *   Button J1- → Pin 3
 *   Button J2+ → Pin 4
 *   Button J2- → Pin 5
 *   Button J3+ → Pin 6
 *   Button J3- → Pin 7
 *   Button J4+ → Pin 8
 *   Button J4- → Pin 9
 *   Button J5+ → Pin 10
 *   Button J5- → Pin 11
 *   Button J6+ → Pin 12
 *   Button J6- → Pin A0
 *   E-Stop    → Pin A1
 *   Status LED → Pin 13 (built-in)
 *
 * UART Protocol:
 *   "J1+"   → Move joint 1 in positive direction
 *   "J1-"   → Move joint 1 in negative direction
 *   "STOP"  → Emergency stop (disable all movement)
 *   "HOME"  → Return to home position
 *
 * Learning Objectives:
 * - Implement button debouncing without delay()
 * - Design simple serial protocols
 * - Handle multiple inputs simultaneously
 * - Understand embedded timing constraints
 */

// =============================================================================
// PIN DEFINITIONS
// =============================================================================

// Button pins (directly wired, directly define the pins)
const int PIN_J1_POS = 2;
const int PIN_J1_NEG = 3;
const int PIN_J2_POS = 4;
const int PIN_J2_NEG = 5;
const int PIN_J3_POS = 6;
const int PIN_J3_NEG = 7;
const int PIN_J4_POS = 8;
const int PIN_J4_NEG = 9;
const int PIN_J5_POS = 10;
const int PIN_J5_NEG = 11;
const int PIN_J6_POS = 12;
const int PIN_J6_NEG = A0;
const int PIN_ESTOP = A1;
const int PIN_LED = 13;

// Number of joints
const int NUM_JOINTS = 6;

// Button array for easier iteration
const int BUTTON_PINS[] = {
  PIN_J1_POS, PIN_J1_NEG,
  PIN_J2_POS, PIN_J2_NEG,
  PIN_J3_POS, PIN_J3_NEG,
  PIN_J4_POS, PIN_J4_NEG,
  PIN_J5_POS, PIN_J5_NEG,
  PIN_J6_POS, PIN_J6_NEG
};
const int NUM_BUTTONS = 12;


// =============================================================================
// CONFIGURATION
// =============================================================================

// Debounce time in milliseconds
// Typical tactile buttons need 20-50ms debounce
const unsigned long DEBOUNCE_DELAY = 50;

// How often to send repeat commands when button held (ms)
// Set to 0 to only send on initial press
const unsigned long REPEAT_DELAY = 100;

// Serial baud rate (must match the ROS2 uart_bridge node)
const long BAUD_RATE = 115200;


// =============================================================================
// STATE VARIABLES
// =============================================================================

// Last stable state of each button (true = pressed)
bool buttonState[NUM_BUTTONS];

// Last raw reading of each button
bool lastReading[NUM_BUTTONS];

// Timestamp of last state change for debouncing
unsigned long lastDebounceTime[NUM_BUTTONS];

// Timestamp of last command sent for repeat functionality
unsigned long lastCommandTime[NUM_BUTTONS];

// Emergency stop state
bool estopActive = false;


// =============================================================================
// SETUP
// =============================================================================

void setup() {
  // Initialize serial communication
  Serial.begin(BAUD_RATE);

  // Configure button pins with internal pull-up resistors
  // Buttons connect pin to GND when pressed (active LOW)
  for (int i = 0; i < NUM_BUTTONS; i++) {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
    buttonState[i] = false;
    lastReading[i] = HIGH;  // Pull-up means HIGH when not pressed
    lastDebounceTime[i] = 0;
    lastCommandTime[i] = 0;
  }

  // E-stop button
  pinMode(PIN_ESTOP, INPUT_PULLUP);

  // Status LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // Send startup message
  Serial.println("READY");
}


// =============================================================================
// MAIN LOOP
// =============================================================================

void loop() {
  // Check emergency stop first
  checkEmergencyStop();

  // If E-stop is active, don't process other buttons
  if (estopActive) {
    // Blink LED rapidly to indicate E-stop
    digitalWrite(PIN_LED, (millis() / 100) % 2);
    return;
  }

  // Process all joint buttons
  for (int i = 0; i < NUM_BUTTONS; i++) {
    processButton(i);
  }

  // Small delay to prevent overwhelming the serial buffer
  delay(1);
}


// =============================================================================
// BUTTON PROCESSING - TODO: Implement these functions
// =============================================================================

/**
 * Process a single button with debouncing.
 *
 * This function should:
 * 1. Read the current button state
 * 2. Apply debouncing logic
 * 3. Detect button press (transition from not-pressed to pressed)
 * 4. Send the appropriate command
 * 5. Handle button repeat if held down
 *
 * @param buttonIndex Index into BUTTON_PINS array (0-11)
 *
 * TODO: Implement this function
 *
 * Debouncing Algorithm:
 * 1. Read the raw pin value
 * 2. If reading differs from lastReading, reset the debounce timer
 * 3. If enough time has passed since last change (DEBOUNCE_DELAY):
 *    - If the stable state has changed, this is a real button event
 *    - Update buttonState
 *    - If button is now pressed, send command
 * 4. Handle repeat: if button still pressed and REPEAT_DELAY has passed,
 *    send command again
 *
 * Hints:
 * - Use millis() for timing (not delay()!)
 * - digitalRead() returns HIGH or LOW
 * - Buttons are active LOW (pressed = LOW because of pull-up)
 * - Use getJointFromButton() and getDirectionFromButton() helpers
 */
void processButton(int buttonIndex) {
  // TODO: Read current button state
  // bool reading = digitalRead(BUTTON_PINS[buttonIndex]);

  // TODO: Implement debouncing logic
  // If reading changed, reset debounce timer
  // if (reading != lastReading[buttonIndex]) {
  //   lastDebounceTime[buttonIndex] = millis();
  // }

  // TODO: Check if debounce time has passed
  // if ((millis() - lastDebounceTime[buttonIndex]) > DEBOUNCE_DELAY) {
  //   // Reading is stable, check if state changed
  //   ...
  // }

  // TODO: Update lastReading for next iteration
  // lastReading[buttonIndex] = reading;

  // Your implementation here...
}


/**
 * Check and handle the emergency stop button.
 *
 * When E-stop is pressed:
 * 1. Send "STOP" command immediately
 * 2. Set estopActive flag
 * 3. Turn on LED
 *
 * When E-stop is released:
 * 1. Clear estopActive flag
 * 2. Send "READY" to indicate system is back online
 *
 * TODO: Implement this function
 *
 * Note: E-stop should have minimal debouncing for safety - we want
 * to respond as quickly as possible to a stop request.
 */
void checkEmergencyStop() {
  // TODO: Read E-stop button (active LOW)
  // bool estopPressed = (digitalRead(PIN_ESTOP) == LOW);

  // TODO: Handle E-stop state changes
  // If newly pressed, send STOP command
  // If released, send READY command

  // Your implementation here...
}


/**
 * Send a joint movement command over serial.
 *
 * @param joint Joint number (1-6)
 * @param direction +1 for positive, -1 for negative
 *
 * TODO: Implement this function
 *
 * Should send commands in the format:
 *   "J1+" for joint 1 positive
 *   "J3-" for joint 3 negative
 *
 * Use Serial.print() and Serial.println()
 */
void sendJointCommand(int joint, int direction) {
  // TODO: Build and send the command string
  // Format: "J" + joint_number + direction_char + newline
  //
  // Example:
  // Serial.print("J");
  // Serial.print(joint);
  // Serial.println(direction > 0 ? "+" : "-");

  // Your implementation here...
}


/**
 * Send the emergency stop command.
 *
 * TODO: Implement this function
 */
void sendStopCommand() {
  // TODO: Send "STOP" over serial
  // Serial.println("STOP");

  // Your implementation here...
}


// =============================================================================
// HELPER FUNCTIONS (PROVIDED)
// =============================================================================

/**
 * Get the joint number (1-6) from a button index.
 *
 * Button indices 0-1 are joint 1, 2-3 are joint 2, etc.
 */
int getJointFromButton(int buttonIndex) {
  return (buttonIndex / 2) + 1;
}


/**
 * Get the direction (+1 or -1) from a button index.
 *
 * Even indices (0, 2, 4...) are positive direction.
 * Odd indices (1, 3, 5...) are negative direction.
 */
int getDirectionFromButton(int buttonIndex) {
  return (buttonIndex % 2 == 0) ? 1 : -1;
}


/**
 * Convert a direction value to a character.
 */
char directionToChar(int direction) {
  return (direction > 0) ? '+' : '-';
}


// =============================================================================
// OPTIONAL: ADVANCED FEATURES
// =============================================================================

/*
 * BONUS CHALLENGES:
 *
 * 1. Add LED feedback:
 *    - Blink LED when any button is pressed
 *    - Different blink patterns for different joints
 *
 * 2. Add speed control:
 *    - Hold button longer = faster movement
 *    - Send speed value with command: "J1+:50" (50% speed)
 *
 * 3. Add combination buttons:
 *    - Press two buttons simultaneously for diagonal movement
 *    - Special combo for "go to home" (e.g., J1+ and J1- together)
 *
 * 4. Add serial command reception:
 *    - Receive acknowledgments from the host
 *    - Implement simple handshaking protocol
 *
 * 5. Save button mapping to EEPROM:
 *    - Allow remapping buttons without reflashing
 */
