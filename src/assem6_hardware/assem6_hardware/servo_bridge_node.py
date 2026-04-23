#!/usr/bin/env python3
"""
ROS2 Servo Bridge Node for Assem6 Robot Arm

This node runs on the Raspberry Pi 4 and bridges ROS2 joint commands
to real servo motors via the PCA9685 driver.

It subscribes to:
  - /joint_states (from barista_gui.py, position_finder.py)
  - /joint_trajectory_controller/joint_trajectory (from MoveIt / run_scenario.py)

It publishes:
  - /servo_joint_states (actual hardware joint positions for feedback)

Architecture:
  [Ubuntu PC]                        [Raspberry Pi 4]
  barista_gui.py ──/joint_states──→  servo_bridge_node.py ──→ PCA9685 ──→ Servos
  MoveIt ──/joint_trajectory──────→
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String
import time
import math
import os
import threading

from ament_index_python.packages import get_package_share_directory
from assem6_hardware.servo_driver import PCA9685ServoDriver


class ServoBridgeNode(Node):
    """ROS2 node that bridges joint commands to servo hardware."""

    def __init__(self):
        super().__init__('servo_bridge')

        # Declare parameters
        self.declare_parameter('config_file', '')
        self.declare_parameter('update_rate', 50.0)
        self.declare_parameter('max_speed', 2.0)  # rad/s
        self.declare_parameter('enable_smoothing', True)
        self.declare_parameter('smoothing_factor', 0.15)  # Lower = smoother but slower

        # Get parameters
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.enable_smoothing = self.get_parameter('enable_smoothing').get_parameter_value().bool_value
        self.smoothing_factor = self.get_parameter('smoothing_factor').get_parameter_value().double_value

        # Resolve config file path
        if not config_file:
            try:
                pkg_share = get_package_share_directory('assem6_hardware')
                config_file = os.path.join(pkg_share, 'config', 'servo_config.yaml')
            except Exception:
                config_file = None

        # Initialize servo driver
        self.get_logger().info('=' * 50)
        self.get_logger().info('SERVO BRIDGE NODE')
        self.get_logger().info('=' * 50)

        self.driver = PCA9685ServoDriver(config_file)
        self.driver.initialize()

        if self.driver.simulation_mode:
            self.get_logger().warn('Running in SIMULATION MODE — no hardware connected')
        else:
            self.get_logger().info('Hardware driver initialized successfully!')

        # Joint names (must match URDF)
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        # Target positions (what we're moving towards)
        self.target_positions = [0.0, 0.0, 0.0, 0.0]

        # Smoothed positions (for gradual movement)
        self.smooth_positions = [0.0, 0.0, 0.0, 0.0]

        # Lock for thread safety
        self.lock = threading.Lock()

        # Whether we've received any command yet
        self.received_command = False

        # ─────────────────────────────────────────────────
        # Subscribers
        # ─────────────────────────────────────────────────

        # Subscribe to joint states (from barista_gui.py, position_finder.py, etc.)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Subscribe to joint trajectory (from MoveIt / run_scenario.py)
        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )

        # Also listen on arm_controller topic
        self.arm_traj_sub = self.create_subscription(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )

        # ─────────────────────────────────────────────────
        # Publishers
        # ─────────────────────────────────────────────────

        # Publish actual servo positions as feedback
        self.servo_state_pub = self.create_publisher(
            JointState,
            '/servo_joint_states',
            10
        )

        # Status publisher
        self.status_pub = self.create_publisher(
            String,
            '/servo_bridge/status',
            10
        )

        # ─────────────────────────────────────────────────
        # Timer for servo updates
        # ─────────────────────────────────────────────────
        timer_period = 1.0 / self.update_rate
        self.update_timer = self.create_timer(timer_period, self.update_servos)

        # Status timer (1 Hz)
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info(f'Update rate: {self.update_rate} Hz')
        self.get_logger().info(f'Max speed: {self.max_speed} rad/s')
        self.get_logger().info(f'Smoothing: {"ON" if self.enable_smoothing else "OFF"} '
                               f'(factor={self.smoothing_factor})')
        self.get_logger().info('Subscribed to: /joint_states, /joint_trajectory_controller/joint_trajectory')
        self.get_logger().info('Publishing to: /servo_joint_states, /servo_bridge/status')
        self.get_logger().info('✓ Servo bridge ready!')
        self.get_logger().info('=' * 50)

    def joint_state_callback(self, msg):
        """Handle incoming joint state messages.

        These come from barista_gui.py at ~50Hz with interpolated positions.
        We extract the position for each of our 4 joints.
        """
        with self.lock:
            for i, name in enumerate(msg.name):
                if name in self.joint_names:
                    idx = self.joint_names.index(name)
                    if idx < len(self.target_positions) and i < len(msg.position):
                        self.target_positions[idx] = msg.position[i]

            self.received_command = True

    def trajectory_callback(self, msg):
        """Handle incoming trajectory messages.

        These come from MoveIt. We extract the final point as the target.
        The actual interpolation is handled by our smoothing loop.
        """
        if not msg.points:
            return

        # Use the last trajectory point as the target
        last_point = msg.points[-1]

        with self.lock:
            for i, name in enumerate(msg.joint_names):
                if name in self.joint_names:
                    idx = self.joint_names.index(name)
                    if idx < len(self.target_positions) and i < len(last_point.positions):
                        self.target_positions[idx] = last_point.positions[i]

            self.received_command = True

        self.get_logger().info(
            f'Trajectory received: target=[{", ".join(f"{p:.2f}" for p in self.target_positions)}]'
        )

    def update_servos(self):
        """Timer callback: update servo positions toward targets.

        If smoothing is enabled, we interpolate gradually using an
        exponential moving average for natural-looking motion.
        Speed is clamped to max_speed for safety.
        """
        if not self.received_command:
            return

        dt = 1.0 / self.update_rate

        with self.lock:
            targets = self.target_positions.copy()

        for i in range(4):
            if self.enable_smoothing:
                # Calculate desired movement
                diff = targets[i] - self.smooth_positions[i]

                # Clamp speed
                max_step = self.max_speed * dt
                if abs(diff) > max_step:
                    diff = max_step if diff > 0 else -max_step

                # Exponential smoothing
                self.smooth_positions[i] += diff * self.smoothing_factor
            else:
                # Direct positioning (no smoothing)
                self.smooth_positions[i] = targets[i]

        # Send to hardware
        self.driver.set_all_angles(self.smooth_positions)

        # Publish feedback
        self.publish_servo_state()

    def publish_servo_state(self):
        """Publish current servo positions as JointState."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.driver.get_angles()
        msg.velocity = [0.0] * 4
        msg.effort = [0.0] * 4
        self.servo_state_pub.publish(msg)

    def publish_status(self):
        """Publish status message (1 Hz)."""
        angles = self.driver.get_angles()
        mode = "SIM" if self.driver.simulation_mode else "HW"
        status = (f"[{mode}] joints=[{', '.join(f'{a:.2f}' for a in angles)}] "
                  f"rate={self.update_rate}Hz")
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def destroy_node(self):
        """Clean shutdown."""
        self.get_logger().info('Shutting down servo bridge...')
        self.driver.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = ServoBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
