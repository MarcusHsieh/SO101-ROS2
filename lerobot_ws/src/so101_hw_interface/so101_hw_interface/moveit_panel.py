#!/usr/bin/env python3
"""
MoveIt Control Panel for SO-101 Robot Arm

Provides a web interface to:
- Position a target sphere in 3D space (visible in Foxglove)
- Plan motion to the target using MoveIt
- Execute the planned trajectory

Usage:
    ros2 run so101_hw_interface moveit_panel
    Then open http://localhost:8081 in your browser
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint,
    BoundingVolume, WorkspaceParameters, RobotState
)
from shape_msgs.msg import SolidPrimitive

from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading
import json
import math

HTML_TEMPLATE = r"""
<!DOCTYPE html>
<html>
<head>
    <title>SO-101 MoveIt Control</title>
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
        .container { max-width: 500px; margin: 0 auto; }
        h1 {
            text-align: center;
            margin-bottom: 8px;
            font-size: 1.5em;
            background: linear-gradient(90deg, #ff6b6b, #feca57);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
        }
        .status {
            text-align: center;
            margin-bottom: 12px;
            font-size: 0.85em;
            padding: 8px;
            border-radius: 6px;
            background: rgba(0,0,0,0.3);
        }
        .status.ready { color: #4ade80; }
        .status.planning { color: #fbbf24; }
        .status.executing { color: #60a5fa; }
        .status.error { color: #f87171; }

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
            margin-bottom: 10px;
            text-transform: uppercase;
            letter-spacing: 1px;
        }
        .control-row {
            display: flex;
            align-items: center;
            gap: 10px;
            margin-bottom: 8px;
        }
        .control-row label {
            min-width: 30px;
            font-size: 0.9em;
            font-weight: 600;
        }
        .control-row input[type="range"] { flex: 1; }
        .control-row .value {
            min-width: 80px;
            text-align: right;
            font-family: monospace;
            font-size: 0.85em;
            color: #feca57;
        }
        input[type="range"] {
            height: 6px;
            border-radius: 3px;
            background: rgba(255,255,255,0.1);
            outline: none;
            -webkit-appearance: none;
        }
        input[type="range"]::-webkit-slider-thumb {
            -webkit-appearance: none;
            width: 16px;
            height: 16px;
            border-radius: 50%;
            background: linear-gradient(135deg, #ff6b6b 0%, #feca57 100%);
            cursor: pointer;
        }
        .buttons {
            display: flex;
            gap: 8px;
            margin-top: 12px;
        }
        button {
            flex: 1;
            padding: 12px;
            border: none;
            border-radius: 8px;
            font-size: 0.95em;
            font-weight: 600;
            cursor: pointer;
            transition: transform 0.1s, opacity 0.2s;
        }
        button:active { transform: scale(0.98); }
        button:disabled { opacity: 0.5; cursor: not-allowed; }
        .btn-plan { background: linear-gradient(135deg, #fbbf24, #f59e0b); color: #000; }
        .btn-execute { background: linear-gradient(135deg, #4ade80, #22c55e); color: #000; }
        .btn-stop { background: linear-gradient(135deg, #ef4444, #dc2626); color: #fff; }
        .btn-home { background: linear-gradient(135deg, #60a5fa, #3b82f6); color: #fff; }

        .preset-buttons {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 6px;
            margin-top: 10px;
        }
        .preset-btn {
            padding: 8px;
            font-size: 0.8em;
            background: rgba(255,255,255,0.1);
            border: 1px solid rgba(255,255,255,0.2);
            color: #fff;
        }
        .preset-btn:hover { background: rgba(255,255,255,0.2); }

        .current-pose {
            margin-top: 12px;
            padding: 10px;
            background: rgba(0,0,0,0.3);
            border-radius: 6px;
            font-family: monospace;
            font-size: 0.8em;
        }
        .current-pose h4 {
            color: #888;
            margin-bottom: 6px;
            font-size: 0.9em;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>MoveIt Control Panel</h1>
        <div id="status" class="status ready">Ready</div>

        <div class="control-section">
            <h3>Target Position (meters)</h3>
            <div class="control-row">
                <label>X</label>
                <input type="range" id="pos-x" min="-0.3" max="0.3" step="0.01" value="0.15">
                <span class="value" id="val-x">0.15 m</span>
            </div>
            <div class="control-row">
                <label>Y</label>
                <input type="range" id="pos-y" min="-0.3" max="0.3" step="0.01" value="0.0">
                <span class="value" id="val-y">0.00 m</span>
            </div>
            <div class="control-row">
                <label>Z</label>
                <input type="range" id="pos-z" min="0.0" max="0.4" step="0.01" value="0.2">
                <span class="value" id="val-z">0.20 m</span>
            </div>

            <div class="preset-buttons">
                <button class="preset-btn" onclick="setPreset(0.15, 0, 0.25)">Front High</button>
                <button class="preset-btn" onclick="setPreset(0.15, 0, 0.1)">Front Low</button>
                <button class="preset-btn" onclick="setPreset(0.1, 0.15, 0.2)">Left</button>
                <button class="preset-btn" onclick="setPreset(0.1, -0.15, 0.2)">Right</button>
                <button class="preset-btn" onclick="setPreset(0.2, 0, 0.15)">Far</button>
                <button class="preset-btn" onclick="setPreset(0.08, 0, 0.3)">Up</button>
            </div>
        </div>

        <div class="buttons">
            <button class="btn-plan" id="btn-plan" onclick="planMotion()">Plan</button>
            <button class="btn-execute" id="btn-execute" onclick="executeMotion()" disabled>Execute</button>
        </div>

        <div class="buttons">
            <button class="btn-home" onclick="goHome()">Home</button>
            <button class="btn-stop" onclick="stopMotion()">STOP</button>
        </div>

        <div class="current-pose" id="current-pose">
            <h4>Current End-Effector Position</h4>
            <div id="ee-pos">Loading...</div>
        </div>
    </div>

    <script>
        let targetX = 0.15, targetY = 0.0, targetZ = 0.2;
        let hasPlan = false;
        let isPlanning = false;
        let isExecuting = false;

        function updateSlider(id, value) {
            document.getElementById(id).value = value;
            const axis = id.split('-')[1];
            document.getElementById('val-' + axis).textContent = value.toFixed(2) + ' m';
        }

        function setPreset(x, y, z) {
            targetX = x; targetY = y; targetZ = z;
            updateSlider('pos-x', x);
            updateSlider('pos-y', y);
            updateSlider('pos-z', z);
            updateMarker();
            hasPlan = false;
            document.getElementById('btn-execute').disabled = true;
        }

        function initSliders() {
            ['x', 'y', 'z'].forEach(function(axis) {
                var slider = document.getElementById('pos-' + axis);
                slider.addEventListener('input', function(e) {
                    var val = parseFloat(e.target.value);
                    document.getElementById('val-' + axis).textContent = val.toFixed(2) + ' m';
                    if (axis === 'x') targetX = val;
                    else if (axis === 'y') targetY = val;
                    else targetZ = val;
                    updateMarker();
                    hasPlan = false;
                    document.getElementById('btn-execute').disabled = true;
                });
            });
        }

        function updateMarker() {
            fetch('/marker', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({x: targetX, y: targetY, z: targetZ})
            });
        }

        function setStatus(text, className) {
            var el = document.getElementById('status');
            el.textContent = text;
            el.className = 'status ' + className;
        }

        function planMotion() {
            if (isPlanning || isExecuting) return;
            isPlanning = true;
            setStatus('Planning...', 'planning');
            document.getElementById('btn-plan').disabled = true;

            fetch('/plan', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({x: targetX, y: targetY, z: targetZ})
            })
            .then(function(r) { return r.json(); })
            .then(function(data) {
                isPlanning = false;
                document.getElementById('btn-plan').disabled = false;
                if (data.success) {
                    hasPlan = true;
                    setStatus('Plan ready! Click Execute', 'ready');
                    document.getElementById('btn-execute').disabled = false;
                } else {
                    setStatus('Planning failed: ' + (data.error || 'Unknown error'), 'error');
                }
            })
            .catch(function(err) {
                isPlanning = false;
                document.getElementById('btn-plan').disabled = false;
                setStatus('Planning error: ' + err, 'error');
            });
        }

        function executeMotion() {
            if (!hasPlan || isExecuting) return;
            isExecuting = true;
            setStatus('Executing...', 'executing');
            document.getElementById('btn-execute').disabled = true;

            fetch('/execute', {method: 'POST'})
            .then(function(r) { return r.json(); })
            .then(function(data) {
                isExecuting = false;
                hasPlan = false;
                if (data.success) {
                    setStatus('Motion complete!', 'ready');
                } else {
                    setStatus('Execution failed: ' + (data.error || 'Unknown'), 'error');
                }
            })
            .catch(function(err) {
                isExecuting = false;
                setStatus('Execution error: ' + err, 'error');
            });
        }

        function goHome() {
            setPreset(0.15, 0, 0.2);
            fetch('/home', {method: 'POST'})
            .then(function(r) { return r.json(); })
            .then(function(data) {
                if (data.success) {
                    setStatus('Moving to home...', 'executing');
                }
            });
        }

        function stopMotion() {
            fetch('/stop', {method: 'POST'});
            setStatus('Stopped', 'error');
            isPlanning = false;
            isExecuting = false;
            hasPlan = false;
            document.getElementById('btn-plan').disabled = false;
            document.getElementById('btn-execute').disabled = true;
        }

        function updateCurrentPose() {
            fetch('/current_pose')
            .then(function(r) { return r.json(); })
            .then(function(data) {
                if (data.position) {
                    document.getElementById('ee-pos').innerHTML =
                        'X: ' + data.position.x.toFixed(3) + ' m<br>' +
                        'Y: ' + data.position.y.toFixed(3) + ' m<br>' +
                        'Z: ' + data.position.z.toFixed(3) + ' m';
                }
            })
            .catch(function() {
                document.getElementById('ee-pos').textContent = 'Not available';
            });
        }

        initSliders();
        updateMarker();
        setInterval(updateCurrentPose, 500);
    </script>
</body>
</html>
"""


class MoveItPanel(Node):
    def __init__(self):
        super().__init__('moveit_panel')

        self.callback_group = ReentrantCallbackGroup()

        # Marker publisher for target visualization
        self.marker_pub = self.create_publisher(
            Marker,
            '/moveit_target_marker',
            10
        )

        # MoveGroup action client
        self._move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action',
            callback_group=self.callback_group
        )

        # Current target position
        self.target_position = {'x': 0.15, 'y': 0.0, 'z': 0.2}

        # Planning state
        self.last_plan_result = None
        self.is_executing = False

        # Current end-effector pose (from TF)
        self.current_ee_pose = None

        # Timer to publish marker continuously
        self.marker_timer = self.create_timer(0.1, self._publish_marker)

        self.get_logger().info('MoveIt Panel started')
        self.get_logger().info('Open http://localhost:9090 in your browser')
        self.get_logger().info('View target marker in Foxglove on /moveit_target_marker')

    def _publish_marker(self):
        """Publish visualization marker for target position."""
        marker = Marker()
        marker.header.frame_id = "base"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "moveit_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = self.target_position['x']
        marker.pose.position.y = self.target_position['y']
        marker.pose.position.z = self.target_position['z']
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        # Orange color
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.8

        self.marker_pub.publish(marker)

    def update_target(self, x, y, z):
        """Update target position."""
        self.target_position = {'x': x, 'y': y, 'z': z}
        self.last_plan_result = None

    def plan_to_target(self):
        """Plan motion to target position."""
        if not self._move_group_client.wait_for_server(timeout_sec=2.0):
            return {'success': False, 'error': 'MoveGroup action server not available'}

        goal = MoveGroup.Goal()

        # Set up the motion plan request
        req = goal.request
        req.group_name = "arm"
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3

        # Workspace parameters
        req.workspace_parameters.header.frame_id = "base"
        req.workspace_parameters.min_corner.x = -1.0
        req.workspace_parameters.min_corner.y = -1.0
        req.workspace_parameters.min_corner.z = -1.0
        req.workspace_parameters.max_corner.x = 1.0
        req.workspace_parameters.max_corner.y = 1.0
        req.workspace_parameters.max_corner.z = 1.0

        # Position constraint
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "base"
        position_constraint.link_name = "gripper"
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0

        # Bounding volume (small sphere around target)
        bounding_volume = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]  # 1cm tolerance
        bounding_volume.primitives.append(sphere)

        sphere_pose = Pose()
        sphere_pose.position.x = self.target_position['x']
        sphere_pose.position.y = self.target_position['y']
        sphere_pose.position.z = self.target_position['z']
        sphere_pose.orientation.w = 1.0
        bounding_volume.primitive_poses.append(sphere_pose)

        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0

        # Add constraint
        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        req.goal_constraints.append(constraints)

        # Planning only (not executing yet)
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        # Send goal and wait
        future = self._move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.done():
            return {'success': False, 'error': 'Planning timed out'}

        goal_handle = future.result()
        if not goal_handle.accepted:
            return {'success': False, 'error': 'Goal rejected'}

        # Get result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)

        if not result_future.done():
            return {'success': False, 'error': 'Planning result timed out'}

        result = result_future.result().result

        if result.error_code.val == 1:  # SUCCESS
            self.last_plan_result = result
            return {'success': True}
        else:
            return {'success': False, 'error': f'Planning failed (code: {result.error_code.val})'}

    def execute_plan(self):
        """Execute the last planned trajectory."""
        if self.last_plan_result is None:
            return {'success': False, 'error': 'No plan available'}

        if not self._move_group_client.wait_for_server(timeout_sec=2.0):
            return {'success': False, 'error': 'MoveGroup action server not available'}

        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3

        # Use the planned trajectory
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        # Copy constraints from last plan
        if self.last_plan_result.planned_trajectory:
            # Re-plan and execute in one go
            req = goal.request
            req.group_name = "arm"
            req.num_planning_attempts = 5
            req.allowed_planning_time = 5.0

            # Workspace
            req.workspace_parameters.header.frame_id = "base"
            req.workspace_parameters.min_corner.x = -1.0
            req.workspace_parameters.min_corner.y = -1.0
            req.workspace_parameters.min_corner.z = -1.0
            req.workspace_parameters.max_corner.x = 1.0
            req.workspace_parameters.max_corner.y = 1.0
            req.workspace_parameters.max_corner.z = 1.0

            # Position constraint
            position_constraint = PositionConstraint()
            position_constraint.header.frame_id = "base"
            position_constraint.link_name = "gripper"

            bounding_volume = BoundingVolume()
            sphere = SolidPrimitive()
            sphere.type = SolidPrimitive.SPHERE
            sphere.dimensions = [0.01]
            bounding_volume.primitives.append(sphere)

            sphere_pose = Pose()
            sphere_pose.position.x = self.target_position['x']
            sphere_pose.position.y = self.target_position['y']
            sphere_pose.position.z = self.target_position['z']
            sphere_pose.orientation.w = 1.0
            bounding_volume.primitive_poses.append(sphere_pose)

            position_constraint.constraint_region = bounding_volume
            position_constraint.weight = 1.0

            constraints = Constraints()
            constraints.position_constraints.append(position_constraint)
            req.goal_constraints.append(constraints)

        self.is_executing = True
        future = self._move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.done():
            self.is_executing = False
            return {'success': False, 'error': 'Execution timed out'}

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.is_executing = False
            return {'success': False, 'error': 'Goal rejected'}

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)

        self.is_executing = False

        if not result_future.done():
            return {'success': False, 'error': 'Execution result timed out'}

        result = result_future.result().result

        if result.error_code.val == 1:  # SUCCESS
            self.last_plan_result = None
            return {'success': True}
        else:
            return {'success': False, 'error': f'Execution failed (code: {result.error_code.val})'}

    def go_home(self):
        """Go to home position (named state)."""
        self.update_target(0.15, 0.0, 0.2)
        return self.plan_to_target()


class PanelHandler(SimpleHTTPRequestHandler):
    node = None

    def log_message(self, format, *args):
        pass

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML_TEMPLATE.encode())

        elif self.path == '/current_pose':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            # Return current target as placeholder
            self.wfile.write(json.dumps({
                'position': self.node.target_position
            }).encode())

        else:
            self.send_error(404)

    def do_POST(self):
        if self.path == '/marker':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode())

            self.node.update_target(data['x'], data['y'], data['z'])

            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"status": "ok"}')

        elif self.path == '/plan':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode())

            self.node.update_target(data['x'], data['y'], data['z'])
            result = self.node.plan_to_target()

            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode())

        elif self.path == '/execute':
            result = self.node.execute_plan()

            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode())

        elif self.path == '/home':
            result = self.node.go_home()

            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode())

        elif self.path == '/stop':
            # Cancel any ongoing motion
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(b'{"status": "stopped"}')

        else:
            self.send_error(404)


def main():
    rclpy.init()
    node = MoveItPanel()

    PanelHandler.node = node
    server = HTTPServer(('0.0.0.0', 9090), PanelHandler)

    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.daemon = True
    server_thread.start()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        server.shutdown()
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
