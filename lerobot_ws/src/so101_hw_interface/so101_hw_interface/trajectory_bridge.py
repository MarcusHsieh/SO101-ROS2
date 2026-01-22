#!/usr/bin/env python3
"""
Trajectory Bridge for SO-101 Robot Arm

Provides FollowJointTrajectory action servers that MoveIt can use to execute
trajectories on the real hardware via motor_bridge.

This bridges between:
- MoveIt's trajectory execution (FollowJointTrajectory action)
- motor_bridge's joint commands (sensor_msgs/JointState)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import time
import threading


class TrajectoryBridge(Node):
    def __init__(self):
        super().__init__('trajectory_bridge')

        self.callback_group = ReentrantCallbackGroup()

        # Publisher to motor_bridge
        self.cmd_pub = self.create_publisher(
            JointState,
            '/so101_follower/joint_commands',
            10
        )

        # Current robot state
        self.current_positions = {}
        self.state_sub = self.create_subscription(
            JointState,
            '/so101_follower/joint_states',
            self._state_callback,
            10
        )

        # Action servers for arm and gripper
        self._arm_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self._execute_trajectory,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group
        )

        self._gripper_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
            execute_callback=self._execute_trajectory,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group
        )

        # Execution state
        self._executing = False
        self._cancel_requested = False

        self.get_logger().info('Trajectory Bridge started')
        self.get_logger().info('  Arm action: /arm_controller/follow_joint_trajectory')
        self.get_logger().info('  Gripper action: /gripper_controller/follow_joint_trajectory')

    def _state_callback(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.current_positions[name] = pos

    def _goal_callback(self, goal_request):
        self.get_logger().info('Received trajectory goal')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        self._cancel_requested = True
        return CancelResponse.ACCEPT

    async def _execute_trajectory(self, goal_handle):
        """Execute a trajectory by interpolating and publishing joint commands."""
        self.get_logger().info('Executing trajectory...')

        self._executing = True
        self._cancel_requested = False

        trajectory = goal_handle.request.trajectory
        joint_names = trajectory.joint_names
        points = trajectory.points

        if len(points) == 0:
            self.get_logger().warn('Empty trajectory received')
            goal_handle.succeed()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            self._executing = False
            return result

        # Get starting time
        start_time = time.time()
        feedback = FollowJointTrajectory.Feedback()

        # Execute each trajectory point
        prev_time = 0.0
        for i, point in enumerate(points):
            if self._cancel_requested:
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                self._executing = False
                self.get_logger().info('Trajectory cancelled')
                return result

            # Calculate time to wait
            point_time = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            wait_time = point_time - prev_time

            if wait_time > 0:
                # Interpolate between current position and target
                num_steps = max(1, int(wait_time / 0.02))  # 50Hz updates
                step_time = wait_time / num_steps

                # Get current positions for interpolation
                start_positions = {}
                for name in joint_names:
                    start_positions[name] = self.current_positions.get(name, 0.0)

                for step in range(num_steps):
                    if self._cancel_requested:
                        break

                    t = (step + 1) / num_steps
                    # Linear interpolation
                    positions = {}
                    for j, name in enumerate(joint_names):
                        start = start_positions[name]
                        end = point.positions[j]
                        positions[name] = start + t * (end - start)

                    # Publish command
                    self._publish_command(positions)

                    # Publish feedback
                    feedback.joint_names = joint_names
                    feedback.actual.positions = [
                        self.current_positions.get(n, 0.0) for n in joint_names
                    ]
                    feedback.desired.positions = list(point.positions)
                    feedback.error.positions = [
                        feedback.desired.positions[j] - feedback.actual.positions[j]
                        for j in range(len(joint_names))
                    ]
                    goal_handle.publish_feedback(feedback)

                    time.sleep(step_time)

            prev_time = point_time

        # Final position
        if len(points) > 0:
            final_point = points[-1]
            positions = {
                name: final_point.positions[j]
                for j, name in enumerate(joint_names)
            }
            self._publish_command(positions)

        self._executing = False
        goal_handle.succeed()

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info('Trajectory completed successfully')
        return result

    def _publish_command(self, positions):
        """Publish joint command to motor_bridge."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(positions.keys())
        msg.position = list(positions.values())
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = TrajectoryBridge()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
