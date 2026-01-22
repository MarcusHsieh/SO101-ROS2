# Week 5: Web GUI Development

## Learning Objectives

By the end of this week, you will be able to:
- Build a simple HTTP server in Python
- Design JSON APIs for robot control
- Handle and validate user input
- Connect web interfaces to ROS2

## Prerequisites

- Completed Week 4 (motor bridge)
- Basic understanding of HTTP and JSON

---

## Part 1: HTTP Basics

### 1.1 How Web GUIs Work

```
┌─────────────┐       HTTP        ┌─────────────┐
│   Browser   │ ◀───────────────▶ │   Python    │
│  (HTML/JS)  │   POST /command   │   Server    │
│             │   {"joint": "1"}  │             │
└─────────────┘                   └──────┬──────┘
                                         │
                                         ▼
                                  ┌─────────────┐
                                  │    ROS2     │
                                  │   Topics    │
                                  └─────────────┘
```

### 1.2 Our API Design

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/` | GET | Serve the HTML page |
| `/joint_command` | POST | Set joint position |
| `/torque` | POST | Enable/disable torque |
| `/state` | GET | Get current positions |
| `/home` | POST | Return to home |

### 1.3 JSON Request/Response

**Request:**
```json
{
    "joint": "1",
    "position": 0.5
}
```

**Response (success):**
```json
{
    "success": true,
    "message": "Joint 1 set to 0.5 rad",
    "clamped": false,
    "final_position": 0.5
}
```

**Response (error):**
```json
{
    "success": false,
    "message": "Invalid joint name: 7"
}
```

---

## Part 2: Python HTTP Server

### 2.1 Basic Structure

```python
from http.server import HTTPServer, BaseHTTPRequestHandler
import json

class MyHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(b"<h1>Hello World</h1>")

    def do_POST(self):
        # Read request body
        content_length = int(self.headers['Content-Length'])
        body = self.rfile.read(content_length)
        data = json.loads(body.decode())

        # Process and respond
        response = {"success": True}
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        self.wfile.write(json.dumps(response).encode())

# Start server
server = HTTPServer(('0.0.0.0', 8080), MyHandler)
server.serve_forever()
```

### 2.2 Integrating with ROS2

The challenge: HTTP server is blocking, ROS2 needs to spin.

**Solution:** Run HTTP server in a separate thread.

```python
import threading

class GUINode(Node):
    def __init__(self):
        super().__init__('gui_node')
        # ... ROS2 setup ...

        # Start HTTP server in background
        self.server = HTTPServer(('0.0.0.0', 8080), MyHandler)
        self.server_thread = threading.Thread(target=self.server.serve_forever)
        self.server_thread.daemon = True
        self.server_thread.start()
```

---

## Part 3: Input Validation

### 3.1 Why Validate?

Users can send:
- Invalid joint names ("7", "arm", "")
- Out-of-range positions (100 radians)
- Wrong data types ("abc" instead of 0.5)
- Missing fields

**Never trust user input!**

### 3.2 Validation Checklist

```python
def handle_joint_command(request_data):
    # 1. Check required fields exist
    if 'joint' not in request_data:
        return {"success": False, "message": "Missing 'joint' field"}

    if 'position' not in request_data:
        return {"success": False, "message": "Missing 'position' field"}

    # 2. Validate joint name
    joint = request_data['joint']
    if joint not in ['1', '2', '3', '4', '5', '6']:
        return {"success": False, "message": f"Invalid joint: {joint}"}

    # 3. Validate position type
    position = request_data['position']
    if not isinstance(position, (int, float)):
        return {"success": False, "message": "Position must be a number"}

    # 4. Clamp to limits
    min_pos, max_pos = JOINT_LIMITS[joint]
    clamped = False
    if position < min_pos:
        position = min_pos
        clamped = True
    elif position > max_pos:
        position = max_pos
        clamped = True

    # 5. Apply the command
    # ... actual robot control ...

    return {
        "success": True,
        "message": f"Joint {joint} set to {position:.3f} rad",
        "clamped": clamped,
        "final_position": position
    }
```

---

## Part 4: Implement the Skeleton

### 4.1 Open the Skeleton

Open `/ros2_ws/src/so101_hw_interface/labs/lab05_gui_endpoint.py`

### 4.2 Implement handle_joint_command

```python
def handle_joint_command(self, request_data: dict) -> dict:
    # Extract joint
    joint = request_data.get('joint')
    if joint is None:
        return {"success": False, "message": "Missing 'joint' field"}

    # Validate joint
    if joint not in JOINTS:
        return {"success": False, "message": f"Invalid joint: {joint}"}

    # Extract position
    position = request_data.get('position')
    if position is None:
        return {"success": False, "message": "Missing 'position' field"}

    # Validate position type
    try:
        position = float(position)
    except (TypeError, ValueError):
        return {"success": False, "message": "Position must be a number"}

    # Clamp to limits
    min_pos, max_pos = JOINT_LIMITS[joint]
    clamped = False
    if position < min_pos or position > max_pos:
        clamped = True
        position = max(min_pos, min(position, max_pos))

    # Update target
    self.target_positions[joint] = position

    return {
        "success": True,
        "message": f"Joint {joint} set to {position:.3f} rad",
        "clamped": clamped,
        "final_position": position
    }
```

### 4.3 Implement handle_torque_request

```python
def handle_torque_request(self, request_data: dict) -> dict:
    enable = request_data.get('enable')
    if enable is None:
        return {"success": False, "message": "Missing 'enable' field"}

    if not isinstance(enable, bool):
        return {"success": False, "message": "'enable' must be boolean"}

    self.torque_enabled = enable

    return {
        "success": True,
        "message": f"Torque {'enabled' if enable else 'disabled'}",
        "torque_enabled": self.torque_enabled
    }
```

### 4.4 Implement handle_get_state

```python
def handle_get_state(self) -> dict:
    return {
        "success": True,
        "joints": self.target_positions.copy(),
        "torque_enabled": self.torque_enabled
    }
```

### 4.5 Run Tests

```bash
cd /ros2_ws/src/so101_hw_interface/labs
python3 lab05_gui_endpoint.py
```

---

## Lab Exercise

### Task 1: Complete the Skeleton

Implement all methods in `lab05_gui_endpoint.py`:
- [ ] `handle_joint_command`
- [ ] `handle_torque_request`
- [ ] `handle_get_state`
- [ ] `handle_home_request`
- [ ] `validate_joint_name` (helper)
- [ ] `clamp_to_limits` (helper)

### Task 2: Add New Endpoints

Design and implement:

1. **`/speed`** - Set movement speed
   - Request: `{"speed": 0.5}` (0.1 to 3.0)
   - Validate range, clamp if needed

2. **`/preset`** - Move to preset position
   - Request: `{"name": "wave"}`
   - Define 3 presets (wave, point, rest)

### Task 3: Error Cases

Write tests for these error cases:
- Empty request body
- Invalid JSON
- Numeric joint name (1 instead of "1")
- Position as string ("0.5")

### Task 4: Test on Real Robot

Run your GUI implementation on the actual robot:

```bash
# This runs YOUR lab05_gui_endpoint.py as the GUI!
./start_all.sh --lab gui
```

Open http://localhost:8082 - that's YOUR code serving the page and handling requests!

Check for errors:
```bash
docker exec so101_ros2 tail -f /tmp/gui.log
```

### Deliverables

- [ ] Completed `lab05_gui_endpoint.py` with all tests passing
- [ ] Implementation of `/speed` and `/preset` endpoints
- [ ] Written documentation of your API
- [ ] Screenshot of YOUR GUI running the robot

---

## Part 5: The Full GUI

### 5.1 How the Real GUI Works

The actual `joint_slider_gui.py` combines:
1. HTTP server (serves HTML + handles API)
2. ROS2 node (publishes commands, subscribes to states)
3. WebSocket (real-time updates)

### 5.2 HTML/JavaScript Integration

```html
<input type="range" id="joint1" min="-3.14" max="3.14" step="0.01"
       onchange="sendCommand('1', this.value)">

<script>
function sendCommand(joint, position) {
    fetch('/joint_command', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({joint: joint, position: parseFloat(position)})
    })
    .then(response => response.json())
    .then(data => {
        if (!data.success) {
            alert('Error: ' + data.message);
        }
    });
}
</script>
```

---

## Common Mistakes

1. **Not returning JSON** - Always set Content-Type header
2. **Forgetting to convert types** - Form data comes as strings
3. **No error handling** - Catch JSON parse errors
4. **Blocking the ROS2 spin** - Use threading

---

## What's Next?

In Week 6, you'll learn:
- Robot calibration procedures
- Saving and loading YAML configuration
- Why calibration matters for accuracy

---

## Reference

### HTTP Status Codes

| Code | Meaning | When to Use |
|------|---------|-------------|
| 200 | OK | Successful request |
| 400 | Bad Request | Invalid input |
| 404 | Not Found | Unknown endpoint |
| 500 | Server Error | Internal failure |

### JSON in Python

```python
import json

# Parse JSON string to dict
data = json.loads('{"key": "value"}')

# Convert dict to JSON string
json_str = json.dumps({"key": "value"})

# Pretty print
json_str = json.dumps(data, indent=2)
```
