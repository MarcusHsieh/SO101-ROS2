# Week 8: Integration Testing

## Learning Objectives

By the end of this week, you will be able to:
- Perform end-to-end system testing
- Debug communication issues systematically
- Add visual feedback to embedded systems
- Document test procedures and results

## Prerequisites

- Completed Week 7 (Arduino + UART bridge)
- Working button controller hardware

---

## Part 1: System Overview

### 1.1 Complete Data Flow

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Full System                                  │
│                                                                      │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐      │
│  │ Arduino  │───▶│  UART    │───▶│  Motor   │───▶│  Servos  │      │
│  │ Buttons  │    │  Bridge  │    │  Bridge  │    │          │      │
│  └──────────┘    └──────────┘    └──────────┘    └──────────┘      │
│       │               │               │               │             │
│    Physical       /dev/ttyUSB1   /joint_commands  /dev/ttyACM0     │
│    Button           Serial         ROS2 Topic       Feetech        │
│    Press                                            Protocol       │
│                                                                      │
│                   Also connected:                                    │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐                      │
│  │  Web GUI │───▶│   HTTP   │───▶│   Same   │                      │
│  │          │    │  Server  │    │  Topics  │                      │
│  └──────────┘    └──────────┘    └──────────┘                      │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### 1.2 What Can Go Wrong

| Layer | Possible Issues |
|-------|-----------------|
| Arduino | Button wiring, debounce timing, serial config |
| USB | Wrong port, permissions, cable issues |
| UART Bridge | Parsing errors, wrong port parameter |
| ROS2 Topics | Wrong topic name, message format |
| Motor Bridge | Conversion errors, calibration issues |
| Servos | Power, wiring, motor IDs |

---

## Part 2: Testing Methodology

### 2.1 Bottom-Up Testing

Test each layer independently before integrating:

```
Layer 1: Arduino alone
    └─▶ Layer 2: Arduino + Serial Monitor
        └─▶ Layer 3: UART Bridge parsing
            └─▶ Layer 4: Full ROS2 integration
                └─▶ Layer 5: Physical robot movement
```

### 2.2 Test Checklist

**Layer 1: Arduino Hardware**
```
□ All buttons click when pressed
□ Multimeter shows continuity button-to-GND when pressed
□ LED turns on/off with digitalWrite test
□ Serial Monitor shows 115200 baud connection
```

**Layer 2: Arduino Serial Output**
```
□ "READY" appears on startup
□ Button press shows "J1+" or similar
□ E-stop shows "STOP"
□ E-stop release shows "READY"
□ No garbage characters in output
```

**Layer 3: UART Bridge Parsing**
```bash
# Run tests
cd /ros2_ws/src/so101_hw_interface/labs
python3 lab07_uart_bridge.py --test
```
```
□ "J1+" parses to ("1", 1)
□ "J3-" parses to ("3", -1)
□ Invalid input returns None
□ Case insensitivity works
```

**Layer 4: ROS2 Integration**
```bash
# In one terminal
ros2 run so101_hw_interface uart_bridge --ros-args -p port:=/dev/ttyUSB1

# In another terminal
ros2 topic echo /so101_follower/joint_commands
```
```
□ Topic receives messages when button pressed
□ Message contains correct joint name
□ Position updates incrementally
□ E-stop stops all updates
```

**Layer 5: Physical Robot**
```
□ Robot moves when button pressed
□ Correct joint moves
□ Correct direction
□ Movement is smooth
□ E-stop stops robot
□ No unexpected movements
```

---

## Part 3: Debugging Techniques

### 3.1 Serial Monitor Debugging

```cpp
// Add debug output to Arduino
void processButton(int index) {
    bool reading = digitalRead(BUTTON_PINS[index]);

    #ifdef DEBUG
    Serial.print("BTN ");
    Serial.print(index);
    Serial.print(": ");
    Serial.println(reading ? "HIGH" : "LOW");
    #endif

    // ... rest of code
}
```

Compile with: `#define DEBUG` at top of file.

### 3.2 ROS2 Topic Debugging

```bash
# List all topics
ros2 topic list

# Check message type
ros2 topic info /so101_follower/joint_commands

# Watch messages in real-time
ros2 topic echo /so101_follower/joint_commands

# Check publish rate
ros2 topic hz /so101_follower/joint_states
```

### 3.3 Serial Port Debugging

```bash
# List serial ports
ls /dev/tty*

# Check port permissions
ls -la /dev/ttyUSB1

# Monitor raw serial data
cat /dev/ttyUSB1

# Or with screen
screen /dev/ttyUSB1 115200
# (Exit with Ctrl+A, then K)
```

### 3.4 Python Serial Debugging

```python
import serial

port = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
while True:
    line = port.readline()
    if line:
        print(f"Received: {repr(line)}")
```

---

## Part 4: Adding Visual Feedback

### 4.1 Arduino LED Patterns

Add visual feedback to know what's happening:

```cpp
// Blink patterns
void blinkLED(int times, int duration) {
    for (int i = 0; i < times; i++) {
        digitalWrite(PIN_LED, HIGH);
        delay(duration);
        digitalWrite(PIN_LED, LOW);
        delay(duration);
    }
}

// Status codes
void indicateStatus(int code) {
    switch (code) {
        case 1:  // Ready
            digitalWrite(PIN_LED, HIGH);
            break;
        case 2:  // Button pressed
            blinkLED(1, 50);
            break;
        case 3:  // E-stop active
            // Fast blink
            digitalWrite(PIN_LED, (millis() / 100) % 2);
            break;
        case 4:  // Error
            blinkLED(5, 100);
            break;
    }
}
```

### 4.2 Multi-LED Setup (Optional)

If you have RGB LEDs or multiple LEDs:

```cpp
const int LED_R = A2;
const int LED_G = A3;
const int LED_B = A4;

void setStatusColor(int r, int g, int b) {
    digitalWrite(LED_R, r);
    digitalWrite(LED_G, g);
    digitalWrite(LED_B, b);
}

// Green = ready, Red = E-stop, Blue = active
void updateStatusLED() {
    if (estopActive) {
        setStatusColor(1, 0, 0);  // Red
    } else if (anyButtonPressed()) {
        setStatusColor(0, 0, 1);  // Blue
    } else {
        setStatusColor(0, 1, 0);  // Green
    }
}
```

---

## Part 5: Performance Tuning

### 5.1 Step Size Adjustment

Find the right balance:

| Step Size | Effect |
|-----------|--------|
| 0.01 rad | Very fine, slow movement |
| 0.05 rad | Good balance (default) |
| 0.1 rad | Fast but less precise |
| 0.2 rad | Very fast, may overshoot |

```python
# In lab07_uart_bridge.py
self.declare_parameter('step_size', 0.05)
self.step_size = self.get_parameter('step_size').value

# Run with different value:
ros2 run so101_hw_interface uart_bridge --ros-args -p step_size:=0.1
```

### 5.2 Debounce Tuning

```cpp
// Too short: false triggers
const unsigned long DEBOUNCE_DELAY = 10;  // Bad

// Too long: feels laggy
const unsigned long DEBOUNCE_DELAY = 200; // Bad

// Just right
const unsigned long DEBOUNCE_DELAY = 50;  // Good
```

### 5.3 Button Repeat Rate

For held buttons:

```cpp
const unsigned long REPEAT_DELAY = 100;  // ms between repeats

if (buttonState[i] == LOW) {
    if (millis() - lastCommandTime[i] > REPEAT_DELAY) {
        sendCommand(i);
        lastCommandTime[i] = millis();
    }
}
```

---

## Lab Exercise

### Task 1: Complete Testing Checklist

Go through all test layers and document results:
- [ ] Layer 1: Hardware verification
- [ ] Layer 2: Serial communication
- [ ] Layer 3: Parsing tests
- [ ] Layer 4: ROS2 topic flow
- [ ] Layer 5: Physical robot

### Task 2: Debug a Problem

Instructor will introduce one of these issues:
- Wrong baud rate
- Swapped button wires
- Incorrect joint mapping
- Missing pull-up resistor

Find and fix it, documenting your process.

### Task 3: Add LED Feedback

Implement status LED that shows:
- Solid: System ready
- Single blink: Button pressed
- Fast blink: E-stop active
- Pattern: Error state

### Task 4: Performance Optimization

1. Measure button-to-movement latency
2. Try different step sizes
3. Document the tradeoffs
4. Choose optimal settings for your setup

### Deliverables

- [ ] Completed test checklist with results
- [ ] Debug log showing problem-solving process
- [ ] Working LED feedback implementation
- [ ] Performance report with measurements

---

## Common Issues and Solutions

| Issue | Likely Cause | Solution |
|-------|--------------|----------|
| No serial output | Wrong baud rate | Check both ends match |
| Garbled text | Baud mismatch | Reset both to 115200 |
| Button always triggers | No pull-up | Use INPUT_PULLUP |
| Button never triggers | Wiring reversed | Check GND connection |
| Wrong joint moves | Pin mapping error | Verify BUTTON_PINS array |
| Movement too slow | Small step size | Increase step_size |
| Robot jerks | Large step size | Decrease step_size |

---

## What's Next?

In Week 9, you'll learn:
- Imitation learning fundamentals
- Leader-follower data collection
- Recording demonstrations

---

## Reference

### Useful Commands

```bash
# Check all connections
ros2 node list
ros2 topic list
ros2 service list

# Monitor specific topic
ros2 topic echo /so101_follower/joint_commands

# Check topic rate
ros2 topic hz /so101_follower/joint_states

# View node info
ros2 node info /uart_bridge

# Call torque service
ros2 service call /so101_follower/set_torque std_srvs/srv/SetBool "{data: false}"
```
