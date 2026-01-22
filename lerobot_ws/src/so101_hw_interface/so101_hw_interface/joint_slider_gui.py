#!/usr/bin/env python3
"""
Web-based Joint Slider GUI for SO-101 Robot Arm

Features:
- Sliders for each joint
- Speed control (max velocity)
- Smoothing control (interpolation factor)
- Real-time position feedback
- Continuous smooth motion

Usage:
    python3 joint_slider_gui.py
    Then open http://localhost:8080 in your browser
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading
import json
import math
import time

JOINT_NAMES = ["1", "2", "3", "4", "5", "6"]
JOINT_LABELS = {
    "1": "Base Rotation",
    "2": "Shoulder",
    "3": "Elbow",
    "4": "Wrist Pitch",
    "5": "Wrist Roll",
    "6": "Gripper"
}

JOINT_LIMITS = {
    "1": (-2.6, 2.6),
    "2": (-2.6, 2.6),
    "3": (-2.6, 2.6),
    "4": (-2.6, 2.6),
    "5": (-2.6, 2.6),
    "6": (-1.0, 1.0),
}

HTML_TEMPLATE = r"""
<!DOCTYPE html>
<html>
<head>
    <title>SO-101 Joint Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            min-height: 100vh;
            padding: 15px;
            color: #fff;
        }
        .container { max-width: 600px; margin: 0 auto; }
        h1 {
            text-align: center;
            margin-bottom: 8px;
            font-size: 1.6em;
            background: linear-gradient(90deg, #00d2ff, #3a7bd5);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
        }
        .status {
            text-align: center;
            margin-bottom: 12px;
            font-size: 0.85em;
            color: #888;
        }
        .status.connected { color: #4ade80; }
        .status.disconnected { color: #f87171; }

        .control-section {
            background: rgba(255,255,255,0.08);
            border-radius: 10px;
            padding: 12px 15px;
            margin-bottom: 12px;
            border: 1px solid rgba(255,255,255,0.15);
        }
        .control-section h3 {
            font-size: 0.8em;
            color: #888;
            margin-bottom: 8px;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        .control-row {
            display: flex;
            align-items: center;
            gap: 12px;
            margin-bottom: 6px;
        }
        .control-row label {
            min-width: 70px;
            font-size: 0.85em;
        }
        .control-row input[type="range"] { flex: 1; }
        .control-row .value {
            min-width: 70px;
            text-align: right;
            font-family: monospace;
            font-size: 0.85em;
            color: #00d2ff;
        }

        .joint-card {
            background: rgba(255,255,255,0.05);
            border-radius: 10px;
            padding: 12px 15px;
            margin-bottom: 8px;
            border: 1px solid rgba(255,255,255,0.1);
        }
        .joint-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 6px;
        }
        .joint-name { font-weight: 600; font-size: 0.95em; }
        .joint-values {
            display: flex;
            gap: 12px;
            font-family: monospace;
            font-size: 0.8em;
        }
        .joint-values .target { color: #00d2ff; }
        .joint-values .current { color: #4ade80; }
        input[type="range"] {
            width: 100%;
            height: 6px;
            border-radius: 3px;
            background: rgba(255,255,255,0.1);
            outline: none;
            -webkit-appearance: none;
        }
        input[type="range"]::-webkit-slider-thumb {
            -webkit-appearance: none;
            width: 18px;
            height: 18px;
            border-radius: 50%;
            background: linear-gradient(135deg, #00d2ff 0%, #3a7bd5 100%);
            cursor: pointer;
            box-shadow: 0 2px 6px rgba(0,210,255,0.3);
        }
        .buttons {
            display: flex;
            gap: 8px;
            margin-top: 12px;
        }
        button {
            flex: 1;
            padding: 10px;
            border: none;
            border-radius: 8px;
            font-size: 0.9em;
            font-weight: 600;
            cursor: pointer;
            transition: transform 0.1s;
        }
        button:active { transform: scale(0.98); }
        .btn-home { background: linear-gradient(135deg, #4ade80, #22c55e); color: #000; }
        .btn-sync { background: linear-gradient(135deg, #f59e0b, #d97706); color: #000; }
        .btn-stop { background: linear-gradient(135deg, #ef4444, #dc2626); color: #fff; }
        .btn-torque-off { background: linear-gradient(135deg, #a855f7, #7c3aed); color: #fff; }
        .btn-torque-on { background: linear-gradient(135deg, #06b6d4, #0891b2); color: #fff; }
        .torque-status {
            text-align: center;
            padding: 8px;
            margin-bottom: 12px;
            border-radius: 6px;
            font-weight: 600;
        }
        .torque-status.enabled { background: rgba(34, 197, 94, 0.2); color: #4ade80; }
        .torque-status.disabled { background: rgba(168, 85, 247, 0.2); color: #a855f7; }
    </style>
</head>
<body>
    <div class="container">
        <h1>SO-101 Joint Control</h1>
        <div id="status" class="status disconnected">Connecting...</div>

        <div class="control-section">
            <h3>Motion Settings</h3>
            <div class="control-row">
                <label>Speed</label>
                <input type="range" id="speed" min="0.1" max="3.0" step="0.1" value="0.8">
                <span class="value" id="speed-val">0.8 rad/s</span>
            </div>
            <div class="control-row">
                <label>Smoothing</label>
                <input type="range" id="smoothing" min="0.02" max="0.5" step="0.02" value="0.15">
                <span class="value" id="smoothing-val">0.15</span>
            </div>
        </div>

        <div id="joints"></div>

        <div class="buttons">
            <button class="btn-home" onclick="goHome()">Home</button>
            <button class="btn-sync" onclick="syncToRobot()">Sync</button>
            <button class="btn-stop" onclick="stopMotion()">STOP</button>
        </div>

        <div id="torque-status" class="torque-status enabled">Torque: ENABLED</div>
        <div class="buttons">
            <button class="btn-torque-off" onclick="setTorque(false)">Torque OFF (Free Move)</button>
            <button class="btn-torque-on" onclick="setTorque(true)">Torque ON</button>
        </div>
    </div>

    <script>
        const joints = JOINT_CONFIG;
        let targetPositions = {};
        let commandPositions = {};
        let robotState = {};
        let lastRobotState = {};
        let speed = 0.8;
        let smoothing = 0.15;
        let isConnected = false;
        let lastStateTime = 0;
        let robotMoving = false;
        let initialized = false;
        let torqueEnabled = true;

        joints.forEach(function(j) {
            targetPositions[j.name] = 0;
            commandPositions[j.name] = 0;
            lastRobotState[j.name] = 0;
        });

        function initSliders() {
            const container = document.getElementById('joints');
            joints.forEach(function(j) {
                const card = document.createElement('div');
                card.className = 'joint-card';
                card.innerHTML =
                    '<div class="joint-header">' +
                    '<span class="joint-name">' + j.label + '</span>' +
                    '<div class="joint-values">' +
                    '<span class="target" id="target-' + j.name + '">T: 0.00</span>' +
                    '<span class="current" id="current-' + j.name + '">C: 0.00</span>' +
                    '</div></div>' +
                    '<input type="range" id="slider-' + j.name + '" ' +
                    'min="' + j.min + '" max="' + j.max + '" step="0.01" value="0">';
                container.appendChild(card);

                document.getElementById('slider-' + j.name).addEventListener('input', function(e) {
                    targetPositions[j.name] = parseFloat(e.target.value);
                    document.getElementById('target-' + j.name).textContent = 'T: ' + parseFloat(e.target.value).toFixed(2);
                });
            });

            document.getElementById('speed').addEventListener('input', function(e) {
                speed = parseFloat(e.target.value);
                document.getElementById('speed-val').textContent = speed.toFixed(1) + ' rad/s';
            });

            document.getElementById('smoothing').addEventListener('input', function(e) {
                smoothing = parseFloat(e.target.value);
                document.getElementById('smoothing-val').textContent = smoothing.toFixed(2);
            });
        }

        function checkTrackingError() {
            // Check if robot is not following commands (large gap between command and actual)
            var maxError = 0;
            joints.forEach(function(j) {
                var error = Math.abs(commandPositions[j.name] - (robotState[j.name] || 0));
                if (error > maxError) maxError = error;
            });
            // If error > 0.3 rad (~17 deg), robot isn't tracking - sync to actual position
            if (maxError > 0.3 && initialized) {
                console.log('Tracking error detected (' + maxError.toFixed(2) + ' rad), syncing to robot');
                joints.forEach(function(j) {
                    commandPositions[j.name] = robotState[j.name] || 0;
                });
            }
        }

        function updateMotion() {
            if (!isConnected) return;

            // Check for tracking errors before updating
            checkTrackingError();

            var hasChange = false;
            var dt = 0.033;
            var maxDelta = speed * dt;

            joints.forEach(function(j) {
                var target = targetPositions[j.name];
                var current = commandPositions[j.name];
                var diff = target - current;

                if (Math.abs(diff) > 0.0005) {
                    var step = diff * smoothing;
                    if (Math.abs(step) > maxDelta) {
                        step = Math.sign(step) * maxDelta;
                    }
                    commandPositions[j.name] = current + step;
                    hasChange = true;
                }
            });

            if (hasChange) {
                sendCommand();
            }
        }

        function sendCommand() {
            fetch('/command', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({positions: commandPositions})
            }).catch(function() {
                // Command failed - will be handled by tracking error detection
            });
        }

        function goHome() {
            joints.forEach(function(j) {
                targetPositions[j.name] = 0;
                document.getElementById('slider-' + j.name).value = 0;
                document.getElementById('target-' + j.name).textContent = 'T: 0.00';
            });
        }

        function syncToRobot() {
            joints.forEach(function(j) {
                var pos = robotState[j.name] || 0;
                targetPositions[j.name] = pos;
                commandPositions[j.name] = pos;
                document.getElementById('slider-' + j.name).value = pos;
                document.getElementById('target-' + j.name).textContent = 'T: ' + pos.toFixed(2);
            });
        }

        function stopMotion() {
            // Stop at current ROBOT position (not command position)
            joints.forEach(function(j) {
                var pos = robotState[j.name] || commandPositions[j.name];
                targetPositions[j.name] = pos;
                commandPositions[j.name] = pos;
                document.getElementById('slider-' + j.name).value = pos;
                document.getElementById('target-' + j.name).textContent = 'T: ' + pos.toFixed(2);
            });
        }

        function updateRobotState() {
            fetch('/state')
                .then(function(r) { return r.json(); })
                .then(function(data) {
                    if (data.positions) {
                        var wasDisconnected = !isConnected;
                        isConnected = true;
                        lastStateTime = Date.now();

                        document.getElementById('status').className = 'status connected';
                        document.getElementById('status').textContent = 'Connected';

                        // Save last state for movement detection
                        lastRobotState = Object.assign({}, robotState);
                        robotState = data.positions;

                        // On first connection or reconnect, sync to robot position
                        if (!initialized || wasDisconnected) {
                            console.log('Initializing/reconnecting - syncing to robot position');
                            joints.forEach(function(j) {
                                var pos = data.positions[j.name] || 0;
                                targetPositions[j.name] = pos;
                                commandPositions[j.name] = pos;
                                document.getElementById('slider-' + j.name).value = pos;
                                document.getElementById('target-' + j.name).textContent = 'T: ' + pos.toFixed(2);
                            });
                            initialized = true;
                        }

                        joints.forEach(function(j) {
                            var pos = data.positions[j.name] || 0;
                            document.getElementById('current-' + j.name).textContent = 'C: ' + pos.toFixed(2);
                        });
                    }
                })
                .catch(function() {
                    isConnected = false;
                    document.getElementById('status').className = 'status disconnected';
                    document.getElementById('status').textContent = 'Disconnected';
                });
        }

        function setTorque(enable) {
            fetch('/torque', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({enable: enable})
            })
            .then(function(r) { return r.json(); })
            .then(function(data) {
                if (data.success) {
                    torqueEnabled = enable;
                    var statusEl = document.getElementById('torque-status');
                    if (enable) {
                        statusEl.className = 'torque-status enabled';
                        statusEl.textContent = 'Torque: ENABLED';
                        // Sync sliders to current robot position when re-enabling
                        syncToRobot();
                    } else {
                        statusEl.className = 'torque-status disabled';
                        statusEl.textContent = 'Torque: DISABLED (Move arm freely)';
                    }
                } else {
                    alert('Torque control failed: ' + (data.error || 'Unknown error'));
                }
            })
            .catch(function(err) {
                alert('Torque control error: ' + err);
            });
        }

        initSliders();
        setInterval(updateMotion, 33);
        setInterval(updateRobotState, 100);
    </script>
</body>
</html>
"""


class JointSliderGUI(Node):
    def __init__(self):
        super().__init__('joint_slider_gui')

        self.publisher = self.create_publisher(
            JointState,
            '/so101_follower/joint_commands',
            10
        )

        self.robot_state = {name: 0.0 for name in JOINT_NAMES}

        self.subscription = self.create_subscription(
            JointState,
            '/so101_follower/joint_states',
            self._state_callback,
            10
        )

        # Torque service client
        self.torque_client = self.create_client(SetBool, '/so101_follower/set_torque')

        self.get_logger().info('Joint Slider GUI started')
        self.get_logger().info('Open http://localhost:8080 in your browser')

    def _state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.robot_state[name] = pos

    def publish_command(self, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(positions.keys())
        msg.position = [float(positions[n]) for n in msg.name]
        self.publisher.publish(msg)

    def set_torque(self, enable):
        """Call the torque service."""
        if not self.torque_client.wait_for_service(timeout_sec=1.0):
            return {'success': False, 'error': 'Torque service not available'}

        request = SetBool.Request()
        request.data = enable

        future = self.torque_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.done():
            response = future.result()
            return {'success': response.success, 'message': response.message}
        else:
            return {'success': False, 'error': 'Service call timed out'}


class GUIHandler(SimpleHTTPRequestHandler):
    node = None

    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()

            joint_config = json.dumps([
                {"name": n, "label": JOINT_LABELS[n],
                 "min": JOINT_LIMITS[n][0], "max": JOINT_LIMITS[n][1]}
                for n in JOINT_NAMES
            ])
            html = HTML_TEMPLATE.replace('JOINT_CONFIG', joint_config)
            self.wfile.write(html.encode())

        elif self.path == '/state':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({
                'positions': self.node.robot_state
            }).encode())
        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == '/command':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode())
            positions = data.get('positions', data)

            self.node.publish_command(positions)

            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"status": "ok"}')

        elif self.path == '/torque':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode())
            enable = data.get('enable', True)

            result = self.node.set_torque(enable)

            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode())

        else:
            self.send_error(404)


def main():
    rclpy.init()
    node = JointSliderGUI()

    GUIHandler.node = node
    server = HTTPServer(('0.0.0.0', 8080), GUIHandler)

    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.daemon = True
    server_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
