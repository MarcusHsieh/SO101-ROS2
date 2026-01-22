# Week 7: Arduino Button Controller

## Learning Objectives

By the end of this week, you will be able to:
- Wire buttons to an Arduino with proper pull-up configuration
- Implement software debouncing
- Design a simple UART protocol
- Create a ROS2 bridge for serial input

## Prerequisites

- Completed Weeks 1-6
- Arduino IDE installed
- Arduino board (Nano, Uno, or similar)

---

## Part 1: Hardware Setup

### 1.1 Components Needed

- 1x Arduino Nano/Uno
- 12x Tactile push buttons (6mm)
- 1x Breadboard
- Jumper wires
- USB cable

### 1.2 Wiring Diagram

```
                        Arduino Nano
                    ┌─────────────────┐
                    │                 │
         J1+ ○──────┤ D2          VIN ├
         J1- ○──────┤ D3          GND ├───○ GND Rail
         J2+ ○──────┤ D4           5V ├───○ 5V Rail
         J2- ○──────┤ D5          ... │
         J3+ ○──────┤ D6              │
         J3- ○──────┤ D7              │
         J4+ ○──────┤ D8              │
         J4- ○──────┤ D9              │
         J5+ ○──────┤ D10             │
         J5- ○──────┤ D11             │
         J6+ ○──────┤ D12             │
         J6- ○──────┤ A0              │
       ESTOP ○──────┤ A1          LED ├───○ (Built-in D13)
                    │                 │
                    └─────────────────┘

    Button Wiring (each button):

         Arduino Pin ○────┬────○ Button ○────○ GND
                          │
                    Internal Pull-up
                    (enabled in code)

    When button pressed: Pin reads LOW
    When button released: Pin reads HIGH (pulled up)
```

### 1.3 Physical Layout

```
    Breadboard Layout:

    ┌──────────────────────────────────────────────────┐
    │  [J1+] [J1-]  [J2+] [J2-]  [J3+] [J3-]          │
    │    ●     ●      ●     ●      ●     ●            │
    │    │     │      │     │      │     │            │
    │  ──┴─────┴──────┴─────┴──────┴─────┴── GND     │
    │                                                  │
    │  [J4+] [J4-]  [J5+] [J5-]  [J6+] [J6-]  [STOP] │
    │    ●     ●      ●     ●      ●     ●      ●    │
    │    │     │      │     │      │     │      │    │
    │  ──┴─────┴──────┴─────┴──────┴─────┴──────┴─── │
    │                                                  │
    │  [====== Arduino Nano ======]                   │
    │                                                  │
    └──────────────────────────────────────────────────┘
```

### 1.4 Testing Connections

Before programming:
1. Use a multimeter in continuity mode
2. Press each button
3. Verify connection between pin and GND

---

## Part 2: Arduino Programming

### 2.1 Pin Configuration

```cpp
const int PIN_J1_POS = 2;
const int PIN_J1_NEG = 3;
// ... etc

void setup() {
    // Enable internal pull-up resistors
    pinMode(PIN_J1_POS, INPUT_PULLUP);
    pinMode(PIN_J1_NEG, INPUT_PULLUP);
    // ... etc

    Serial.begin(115200);
}
```

### 2.2 Button Debouncing

**The Problem:** Buttons "bounce" - rapid on/off transitions when pressed.

```
Physical press:   ▔▔▔▔\____/▔▔▔▔
What Arduino sees: ▔▔▔▔\⌇⌇⌇/▔▔▔▔  (multiple transitions)
```

**The Solution:** Ignore changes for a short time after detecting one.

```cpp
const unsigned long DEBOUNCE_DELAY = 50;  // milliseconds

bool lastReading[12];
bool buttonState[12];
unsigned long lastDebounceTime[12];

void processButton(int index) {
    bool reading = digitalRead(BUTTON_PINS[index]);

    // If reading changed, reset timer
    if (reading != lastReading[index]) {
        lastDebounceTime[index] = millis();
    }

    // If stable for DEBOUNCE_DELAY, accept as real change
    if ((millis() - lastDebounceTime[index]) > DEBOUNCE_DELAY) {
        if (reading != buttonState[index]) {
            buttonState[index] = reading;

            // Button pressed (LOW because of pull-up)
            if (buttonState[index] == LOW) {
                sendCommand(index);
            }
        }
    }

    lastReading[index] = reading;
}
```

### 2.3 UART Protocol

Simple text-based protocol:

| Command | Meaning |
|---------|---------|
| `J1+` | Move joint 1 positive |
| `J1-` | Move joint 1 negative |
| `J2+` | Move joint 2 positive |
| `STOP` | Emergency stop |
| `HOME` | Go to home position |

```cpp
void sendCommand(int buttonIndex) {
    int joint = (buttonIndex / 2) + 1;  // 0-1->1, 2-3->2, etc.
    char direction = (buttonIndex % 2 == 0) ? '+' : '-';

    Serial.print("J");
    Serial.print(joint);
    Serial.println(direction);  // Adds \n
}
```

---

## Part 3: Arduino Skeleton

### 3.1 Open the Skeleton

Open `/home/mj/SO101-ROS2/hardware/arduino/button_controller/button_controller.ino`

### 3.2 Key Functions to Implement

**processButton:**
```cpp
void processButton(int buttonIndex) {
    // TODO: Read button state
    bool reading = digitalRead(BUTTON_PINS[buttonIndex]);

    // TODO: Debounce logic
    if (reading != lastReading[buttonIndex]) {
        lastDebounceTime[buttonIndex] = millis();
    }

    if ((millis() - lastDebounceTime[buttonIndex]) > DEBOUNCE_DELAY) {
        if (reading != buttonState[buttonIndex]) {
            buttonState[buttonIndex] = reading;

            if (buttonState[buttonIndex] == LOW) {  // Pressed
                int joint = getJointFromButton(buttonIndex);
                int direction = getDirectionFromButton(buttonIndex);
                sendJointCommand(joint, direction);
            }
        }
    }

    lastReading[buttonIndex] = reading;
}
```

**sendJointCommand:**
```cpp
void sendJointCommand(int joint, int direction) {
    Serial.print("J");
    Serial.print(joint);
    Serial.println(direction > 0 ? "+" : "-");
}
```

**checkEmergencyStop:**
```cpp
void checkEmergencyStop() {
    bool pressed = (digitalRead(PIN_ESTOP) == LOW);

    if (pressed && !estopActive) {
        estopActive = true;
        Serial.println("STOP");
        digitalWrite(PIN_LED, HIGH);
    }
    else if (!pressed && estopActive) {
        estopActive = false;
        Serial.println("READY");
        digitalWrite(PIN_LED, LOW);
    }
}
```

### 3.3 Upload and Test

1. Open Arduino IDE
2. Select your board (Arduino Nano)
3. Select the correct port
4. Upload the sketch
5. Open Serial Monitor (115200 baud)
6. Press buttons - you should see commands appear

---

## Part 4: ROS2 UART Bridge

### 4.1 Open the Skeleton

Open `/ros2_ws/src/so101_hw_interface/labs/lab07_uart_bridge.py`

### 4.2 Parse Commands

```python
def parse_joint_command(self, command: str) -> Optional[Tuple[str, int]]:
    command = command.upper().strip()

    if len(command) < 3:
        return None

    if command[0] != 'J':
        return None

    joint = command[1:-1]
    if joint not in JOINTS:
        return None

    direction_char = command[-1]
    if direction_char == '+':
        return (joint, 1)
    elif direction_char == '-':
        return (joint, -1)

    return None
```

### 4.3 Process Commands

```python
def _process_command(self, command: str):
    command = command.upper().strip()

    if command == "STOP":
        self.handle_estop()
        return

    if command == "HOME":
        self.handle_home()
        return

    if command == "READY":
        self.estop_active = False
        self.get_logger().info("System ready")
        return

    # Parse joint command
    result = self.parse_joint_command(command)
    if result and not self.estop_active:
        joint, direction = result
        self.update_joint_position(joint, direction)
```

### 4.4 Update Position

```python
def update_joint_position(self, joint: str, direction: int):
    current = self.joint_positions[joint]
    new_pos = current + (direction * self.step_size)

    # Clamp to limits
    new_pos = max(MIN_POSITION, min(MAX_POSITION, new_pos))

    self.joint_positions[joint] = new_pos
    self.publish_joint_command()
```

---

## Part 5: Integration

### 5.1 Connect Everything

```
┌─────────────┐       USB        ┌─────────────┐       USB        ┌─────────────┐
│   Arduino   │ ───────────────▶ │   Laptop    │ ◀─────────────── │  Robot Arm  │
│  + Buttons  │   /dev/ttyUSB1   │   Docker    │   /dev/ttyACM0   │  (Servos)   │
└─────────────┘                  └──────┬──────┘                  └─────────────┘
                                        │
                                   uart_bridge
                                        │
                                        ▼
                                 /joint_commands
                                        │
                                        ▼
                                  motor_bridge
```

### 5.2 Run the System

```bash
# Terminal 1: Start robot
./start_all.sh

# Terminal 2: Start UART bridge
ros2 run so101_hw_interface uart_bridge --ros-args -p port:=/dev/ttyUSB1
```

### 5.3 Test

1. Press J1+ button → Joint 1 should move positive
2. Press J1- button → Joint 1 should move negative
3. Press ESTOP → Robot should stop, LED on
4. Release ESTOP → Robot ready again

---

## Lab Exercise

### Task 1: Complete Arduino Sketch

Implement in `button_controller.ino`:
- [ ] `processButton()` with debouncing
- [ ] `sendJointCommand()`
- [ ] `checkEmergencyStop()`
- [ ] `sendStopCommand()`

### Task 2: Complete UART Bridge

Implement in `uart_bridge.py`:
- [ ] `parse_joint_command()`
- [ ] `update_joint_position()`
- [ ] `publish_joint_command()`
- [ ] `handle_estop()`
- [ ] `handle_home()`

### Task 3: Full Integration

1. Wire up at least 4 buttons (2 joints)
2. Upload Arduino sketch
3. Run UART bridge
4. Demonstrate control of robot via buttons

### Deliverables

- [ ] Completed Arduino sketch
- [ ] Completed UART bridge
- [ ] Photo of your button wiring
- [ ] Video of controlling robot with buttons

---

## Troubleshooting

### Arduino Not Detected

- Check USB cable (some are charge-only)
- Install CH340 driver if needed
- Try different USB port

### Commands Not Received

- Check baud rate matches (115200)
- Verify correct serial port
- Check for loose wires

### Buttons Not Working

- Test with multimeter
- Check INPUT_PULLUP is enabled
- Verify correct pin numbers

---

## What's Next?

In Week 8, you'll:
- Perform full system integration testing
- Debug communication issues
- Add visual feedback (LEDs)

---

## Reference

### Arduino Functions

```cpp
pinMode(pin, INPUT_PULLUP);  // Enable internal pull-up
digitalRead(pin);             // Returns HIGH or LOW
digitalWrite(pin, HIGH);      // Set pin high
millis();                     // Milliseconds since startup
Serial.begin(115200);         // Initialize serial
Serial.print("text");         // Print without newline
Serial.println("text");       // Print with newline
```

### pyserial in Python

```python
import serial

# Open port
port = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1)

# Read line
line = port.readline().decode('utf-8').strip()

# Write
port.write(b"command\n")

# Close
port.close()
```
