# Arduino Button Controller - Wiring Diagram

## Overview

This document describes how to wire buttons to an Arduino for controlling the SO-101 robot arm.

## Components Required

| Component | Quantity | Notes |
|-----------|----------|-------|
| Arduino Nano/Uno | 1 | Any 5V Arduino works |
| Tactile Push Buttons | 12 | 6mm recommended |
| Breadboard | 1 | Half-size or larger |
| Jumper Wires | ~20 | Male-to-male |
| USB Cable | 1 | Type-B or Mini-B depending on Arduino |

## Pin Assignments

| Function | Arduino Pin | Button |
|----------|-------------|--------|
| Joint 1 Positive | D2 | J1+ |
| Joint 1 Negative | D3 | J1- |
| Joint 2 Positive | D4 | J2+ |
| Joint 2 Negative | D5 | J2- |
| Joint 3 Positive | D6 | J3+ |
| Joint 3 Negative | D7 | J3- |
| Joint 4 Positive | D8 | J4+ |
| Joint 4 Negative | D9 | J4- |
| Joint 5 Positive | D10 | J5+ |
| Joint 5 Negative | D11 | J5- |
| Joint 6 Positive | D12 | J6+ |
| Joint 6 Negative | A0 | J6- |
| Emergency Stop | A1 | E-STOP |
| Status LED | D13 | (Built-in) |

## Schematic

```
                            ARDUINO NANO
                     ┌──────────────────────┐
                     │ D13 ●────────── LED  │
                     │ 3V3                  │
                     │ REF                  │
     J1+ ──────────● │ A0 ────────────● J6- │
     J1- ──────────● │ A1 ────────────● ESTOP
     J2+ ──────────● │ A2                   │
     J2- ──────────● │ A3                   │
     J3+ ──────────● │ A4                   │
     J3- ──────────● │ A5                   │
     J4+ ──────────● │ A6                   │
     J4- ──────────● │ A7                   │
                     │ 5V ─────────────● +5V Rail
                     │ RST                  │
                     │ GND ────────────● GND Rail
                     │ VIN                  │
                     │                      │
     J5+ ──────────● │ D2                   │
     J5- ──────────● │ D3                   │
     J6+ ──────────● │ D4                   │
                     │ D5                   │
                     │ D6                   │
                     │ D7                   │
                     │ D8                   │
                     │ D9                   │
                     │ D10                  │
                     │ D11                  │
                     │ D12                  │
                     └──────────────────────┘

    BUTTON WIRING DETAIL:
    ═══════════════════════

    Arduino Pin ●─────┬─────●  ●─────● GND Rail
                      │     Button
                      │
                 Internal
                 Pull-up
                 Resistor
                 (enabled in code)

    When button is NOT pressed:
        Pin reads HIGH (pulled up to 5V internally)

    When button IS pressed:
        Pin reads LOW (connected to GND through button)
```

## Breadboard Layout

```
    ┌──────────────────────────────────────────────────────────────────┐
    │                         BREADBOARD TOP VIEW                       │
    │                                                                   │
    │  + ═══════════════════════════════════════════════════════════ + │
    │  - ═══════════════════════════════════════════════════════════ - │
    │                                                                   │
    │     1   5   10   15   20   25   30   35   40   45   50   55  60  │
    │  a  ●   ●   ●    ●    ●    ●    ●    ●    ●    ●    ●    ●   ●  │
    │  b  ●   ●   ●    ●    ●    ●    ●    ●    ●    ●    ●    ●   ●  │
    │  c  ●   ●   ●    ●    ●    ●    ●    ●    ●    ●    ●    ●   ●  │
    │  d  ●   ●   ●    ●    ●    ●    ●    ●    ●    ●    ●    ●   ●  │
    │  e  ●   ●   ●    ●    ●    ●    ●    ●    ●    ●    ●    ●   ●  │
    │     ════════════════════════════════════════════════════════════ │
    │  f  ●   ●   ●    ●    ●    ●    ●    ●    ●    ●    ●    ●   ●  │
    │  g  ●   ●   ●    ●    ●    ●    ●    ●    ●    ●    ●    ●   ●  │
    │  h  ●   ●   ●    ●    ●    ●    ●    ●    ●    ●    ●    ●   ●  │
    │  i  ●   ●   ●    ●    ●    ●    ●    ●    ●    ●    ●    ●   ●  │
    │  j  ●   ●   ●    ●    ●    ●    ●    ●    ●    ●    ●    ●   ●  │
    │                                                                   │
    │  + ═══════════════════════════════════════════════════════════ + │
    │  - ═══════════════════════════════════════════════════════════ - │
    └──────────────────────────────────────────────────────────────────┘

    COMPONENT PLACEMENT:
    ════════════════════

    Arduino Nano: Columns 25-45, spanning center gap

    Buttons (6mm tactile):
    ┌───────────────────────────────────────────────────────────┐
    │  Column:  5    10    15    20    47    52    57          │
    │                                                           │
    │  Row a-c: [J1+] [J1-] [J2+] [J2-]                        │
    │  Row f-h:                         [J3+] [J3-] [ESTOP]    │
    │                                                           │
    │  Row a-c:                         [J4+] [J4-]            │
    │  Row f-h: [J5+] [J5-] [J6+] [J6-]                        │
    └───────────────────────────────────────────────────────────┘

    WIRE COLORS (suggested):
    ═══════════════════════
    • Red: 5V power
    • Black: GND
    • Blue: Joint positive buttons
    • Yellow: Joint negative buttons
    • Orange: Emergency stop
```

## Wiring Steps

### Step 1: Place the Arduino

1. Insert Arduino Nano into breadboard
2. Position it spanning the center gap
3. Ensure all pins are accessible

### Step 2: Connect Power Rails

1. Connect Arduino GND to breadboard GND rail (black wire)
2. Connect Arduino 5V to breadboard + rail (red wire)

### Step 3: Place Buttons

For each button:
1. Place button across the center gap (legs on both sides)
2. Button should straddle rows (e.g., rows c-f)

### Step 4: Connect Buttons to GND

1. One leg of each button connects to GND rail
2. Use jumper wires from button leg to GND rail

### Step 5: Connect Buttons to Arduino

1. Connect other leg of each button to Arduino pin
2. Follow the pin assignment table above

### Step 6: Verify Connections

Before powering on:
1. Use multimeter in continuity mode
2. Press each button
3. Verify continuity between Arduino pin and GND when pressed
4. Verify NO continuity when released

## Testing

### Hardware Test

1. Upload this test sketch:

```cpp
void setup() {
    Serial.begin(115200);
    for (int pin = 2; pin <= 12; pin++) {
        pinMode(pin, INPUT_PULLUP);
    }
    pinMode(A0, INPUT_PULLUP);
    pinMode(A1, INPUT_PULLUP);
}

void loop() {
    Serial.print("Pins: ");
    for (int pin = 2; pin <= 12; pin++) {
        Serial.print(digitalRead(pin));
        Serial.print(" ");
    }
    Serial.print(digitalRead(A0));
    Serial.print(" ");
    Serial.print(digitalRead(A1));
    Serial.println();
    delay(100);
}
```

2. Open Serial Monitor (115200 baud)
3. You should see all 1s (HIGH due to pull-ups)
4. Press each button - corresponding position should show 0

### Expected Output

```
Pins: 1 1 1 1 1 1 1 1 1 1 1 1 1    <- All released
Pins: 0 1 1 1 1 1 1 1 1 1 1 1 1    <- J1+ pressed
Pins: 1 0 1 1 1 1 1 1 1 1 1 1 1    <- J1- pressed
...
```

## Troubleshooting

### Button Not Detected

| Symptom | Cause | Solution |
|---------|-------|----------|
| Always reads 1 | Button not connected | Check wiring |
| Always reads 0 | Short to GND | Check for solder bridges |
| Intermittent | Loose connection | Reseat wires |

### Multiple Buttons Trigger Together

- Check for crossed wires
- Ensure each button has separate GND connection
- Look for shorts on breadboard

### No Serial Output

- Check USB cable is data-capable (not charge-only)
- Verify correct COM port selected
- Check baud rate matches (115200)

## Alternative: Minimal Setup

If you have fewer buttons, start with just 4:

| Function | Pin |
|----------|-----|
| Joint 1+ | D2 |
| Joint 1- | D3 |
| Joint 2+ | D4 |
| Joint 2- | D5 |

This lets you test the system before wiring all 12 buttons.

## Photos Reference

Take photos of your wiring at each stage for debugging:
1. Arduino placed
2. Power rails connected
3. First button wired
4. All buttons complete
5. Final verification

## Safety Notes

- Always disconnect USB before modifying wiring
- Don't exceed 5V on any input pin
- Buttons rated for 12V/50mA are sufficient
- Use appropriate wire gauge (22-24 AWG)
