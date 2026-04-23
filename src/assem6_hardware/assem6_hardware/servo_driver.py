#!/usr/bin/env python3
"""
PCA9685 Servo Driver for Assem6 Robot Arm

Low-level driver that converts joint angles (radians) to PWM signals
for PCA9685 I2C servo controller on Raspberry Pi 4.

Servo Specifications:
  Joint 1: 80 kg.cm, 270° range servo
  Joint 2: 150 kg.cm, 180° range servo
  Joint 3: 80 kg.cm, 180° range servo
  Joint 4: 40 kg.cm, 180° range servo
"""

import math
import time
import yaml
import os

# Try to import hardware libraries — will fail gracefully on non-Pi systems
try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685
    from adafruit_motor import servo as adafruit_servo
    HAS_PCA9685 = True
except ImportError:
    HAS_PCA9685 = False

try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False


class ServoJoint:
    """Represents a single servo joint with calibration parameters."""

    def __init__(self, name, channel, min_pulse_us, max_pulse_us,
                 min_angle_rad, max_angle_rad, offset_rad=0.0,
                 inverted=False, default_position_rad=0.0,
                 servo_range_deg=180):
        self.name = name
        self.channel = channel
        self.min_pulse_us = min_pulse_us
        self.max_pulse_us = max_pulse_us
        self.min_angle_rad = min_angle_rad
        self.max_angle_rad = max_angle_rad
        self.offset_rad = offset_rad
        self.inverted = inverted
        self.default_position_rad = default_position_rad
        self.servo_range_deg = servo_range_deg

        # Current state
        self.current_angle_rad = default_position_rad
        self.target_angle_rad = default_position_rad

    def angle_to_pulse(self, angle_rad):
        """Convert angle in radians to PWM pulse width in microseconds.

        The servo's physical range is mapped linearly:
          min_angle_rad → min_pulse_us
          max_angle_rad → max_pulse_us
        """
        # Clamp to joint limits
        angle_rad = max(self.min_angle_rad, min(self.max_angle_rad, angle_rad))

        # Apply calibration offset
        angle_rad += self.offset_rad

        # Re-clamp after offset
        angle_rad = max(self.min_angle_rad, min(self.max_angle_rad, angle_rad))

        if self.inverted:
            # Reverse the mapping
            ratio = (self.max_angle_rad - angle_rad) / (self.max_angle_rad - self.min_angle_rad)
        else:
            ratio = (angle_rad - self.min_angle_rad) / (self.max_angle_rad - self.min_angle_rad)

        pulse_us = self.min_pulse_us + ratio * (self.max_pulse_us - self.min_pulse_us)
        return int(pulse_us)

    def pulse_to_angle(self, pulse_us):
        """Convert PWM pulse width in microseconds to angle in radians."""
        ratio = (pulse_us - self.min_pulse_us) / (self.max_pulse_us - self.min_pulse_us)

        if self.inverted:
            angle_rad = self.max_angle_rad - ratio * (self.max_angle_rad - self.min_angle_rad)
        else:
            angle_rad = self.min_angle_rad + ratio * (self.max_angle_rad - self.min_angle_rad)

        angle_rad -= self.offset_rad
        return angle_rad

    def __repr__(self):
        return (f"ServoJoint({self.name}, ch={self.channel}, "
                f"range=[{math.degrees(self.min_angle_rad):.0f}°, "
                f"{math.degrees(self.max_angle_rad):.0f}°], "
                f"servo={self.servo_range_deg}°)")


class PCA9685ServoDriver:
    """
    Manages a PCA9685 I2C servo controller with per-joint calibration.

    Usage:
        driver = PCA9685ServoDriver('config/servo_config.yaml')
        driver.initialize()
        driver.set_angle('joint1', 0.5)   # Set joint1 to 0.5 radians
        driver.set_all_angles([0.0, 0.5, -0.3, 0.0])
        driver.shutdown()
    """

    def __init__(self, config_path=None):
        self.joints = {}  # name -> ServoJoint
        self.joint_order = ['joint1', 'joint2', 'joint3', 'joint4']
        self.pca = None
        self.servos = {}  # channel -> PCA9685 servo object
        self.initialized = False
        self.simulation_mode = False

        # Hardware config defaults
        self.i2c_bus = 1
        self.i2c_address = 0x40
        self.pwm_frequency = 50
        self.update_rate = 50
        self.max_speed_rad_s = 1.0  # Max angular speed for safety

        if config_path:
            self.load_config(config_path)
        else:
            self._create_default_config()

    def _create_default_config(self):
        """Create default joint configuration matching Assem6 hardware specs."""
        # Joint 1: 80 kg.cm, 270° range servo
        # Maps -135° to +135° (-2.356 to +2.356 rad)
        self.joints['joint1'] = ServoJoint(
            name='joint1',
            channel=15,
            min_pulse_us=500,
            max_pulse_us=2500,
            min_angle_rad=-2.356,  # -135°
            max_angle_rad=2.356,   # +135°
            servo_range_deg=270,
        )

        # Joint 2: 150 kg.cm, 180° range servo
        # Maps -90° to +90° (-1.57 to +1.57 rad)
        self.joints['joint2'] = ServoJoint(
            name='joint2',
            channel=14,
            min_pulse_us=500,
            max_pulse_us=2500,
            min_angle_rad=-1.5708,  # -90°
            max_angle_rad=1.5708,   # +90°
            servo_range_deg=180,
        )

        # Joint 3: 80 kg.cm, 180° range servo
        # Maps -90° to +90° (-1.57 to +1.57 rad)
        self.joints['joint3'] = ServoJoint(
            name='joint3',
            channel=13,
            min_pulse_us=500,
            max_pulse_us=2500,
            min_angle_rad=-1.5708,
            max_angle_rad=1.5708,
            servo_range_deg=180,
        )

        # Joint 4: 40 kg.cm, 180° range servo
        # Maps -90° to +90° (-1.57 to +1.57 rad)
        self.joints['joint4'] = ServoJoint(
            name='joint4',
            channel=10,
            min_pulse_us=500,
            max_pulse_us=2500,
            min_angle_rad=-1.5708,
            max_angle_rad=1.5708,
            servo_range_deg=180,
        )

    def load_config(self, config_path):
        """Load joint configuration from YAML file."""
        if not os.path.exists(config_path):
            print(f"[ServoDriver] Config file not found: {config_path}")
            print("[ServoDriver] Using default configuration")
            self._create_default_config()
            return

        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        # Load hardware settings
        hw = config.get('hardware', {})
        self.i2c_bus = hw.get('i2c_bus', 1)
        self.i2c_address = hw.get('i2c_address', 0x40)
        self.pwm_frequency = hw.get('pwm_frequency', 50)
        self.update_rate = hw.get('update_rate', 50)
        self.max_speed_rad_s = hw.get('max_speed', 1.0)

        # Load servo/joint settings
        servos_config = config.get('servos', {})
        for joint_name in self.joint_order:
            if joint_name in servos_config:
                jc = servos_config[joint_name]
                self.joints[joint_name] = ServoJoint(
                    name=joint_name,
                    channel=jc.get('channel', self.joint_order.index(joint_name)),
                    min_pulse_us=jc.get('min_pulse', 500),
                    max_pulse_us=jc.get('max_pulse', 2500),
                    min_angle_rad=jc.get('min_angle', -1.5708),
                    max_angle_rad=jc.get('max_angle', 1.5708),
                    offset_rad=jc.get('offset', 0.0),
                    inverted=jc.get('inverted', False),
                    default_position_rad=jc.get('default_position', 0.0),
                    servo_range_deg=jc.get('servo_range_deg', 180),
                )

        # Fill in any missing joints with defaults
        if len(self.joints) < 4:
            self._create_default_config()

        print(f"[ServoDriver] Loaded config from {config_path}")
        for name, joint in self.joints.items():
            print(f"  {joint}")

    def initialize(self):
        """Initialize PCA9685 hardware."""
        if not HAS_PCA9685:
            print("[ServoDriver] WARNING: PCA9685 library not available!")
            print("[ServoDriver] Running in SIMULATION MODE (no hardware)")
            print("[ServoDriver] Install with: pip3 install adafruit-circuitpython-pca9685 adafruit-circuitpython-motor")
            self.simulation_mode = True
            self.initialized = True
            return True

        try:
            # Initialize I2C and PCA9685
            i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = PCA9685(i2c, address=self.i2c_address)
            self.pca.frequency = self.pwm_frequency

            print(f"[ServoDriver] PCA9685 initialized on I2C bus")
            print(f"  Address: 0x{self.i2c_address:02X}")
            print(f"  PWM Frequency: {self.pwm_frequency} Hz")

            # Move all servos to default positions smoothly
            print("[ServoDriver] Moving to default positions...")
            for name in self.joint_order:
                joint = self.joints[name]
                self._set_servo_pulse(joint.channel,
                                      joint.angle_to_pulse(joint.default_position_rad))
                joint.current_angle_rad = joint.default_position_rad
                time.sleep(0.1)

            self.initialized = True
            print("[ServoDriver] ✓ Hardware initialized successfully!")
            return True

        except Exception as e:
            print(f"[ServoDriver] ERROR initializing PCA9685: {e}")
            print("[ServoDriver] Falling back to SIMULATION MODE")
            self.simulation_mode = True
            self.initialized = True
            return False

    def _set_servo_pulse(self, channel, pulse_us):
        """Set raw PWM pulse width on a PCA9685 channel.

        PCA9685 has 12-bit resolution (0-4095) for the pulse within
        each PWM period. At 50Hz, the period is 20000µs.
        """
        if self.simulation_mode or self.pca is None:
            return

        # Convert microseconds to PCA9685 duty cycle (0-65535 for 16-bit)
        # Period at 50Hz = 20000µs
        period_us = 1_000_000.0 / self.pwm_frequency
        duty_cycle = int((pulse_us / period_us) * 65535)
        duty_cycle = max(0, min(65535, duty_cycle))

        try:
            self.pca.channels[channel].duty_cycle = duty_cycle
        except Exception as e:
            print(f"[ServoDriver] Error setting channel {channel}: {e}")

    def set_angle(self, joint_name, angle_rad):
        """Set a single joint to the specified angle (radians).

        Returns the actual angle after clamping to joint limits.
        """
        if joint_name not in self.joints:
            print(f"[ServoDriver] Unknown joint: {joint_name}")
            return None

        if not self.initialized:
            print("[ServoDriver] Driver not initialized! Call initialize() first.")
            return None

        joint = self.joints[joint_name]

        # Clamp to hardware limits
        clamped_angle = max(joint.min_angle_rad, min(joint.max_angle_rad, angle_rad))

        if abs(clamped_angle - angle_rad) > 0.01:
            print(f"[ServoDriver] {joint_name}: angle {math.degrees(angle_rad):.1f}° "
                  f"clamped to {math.degrees(clamped_angle):.1f}°")

        # Convert to PWM and send
        pulse_us = joint.angle_to_pulse(clamped_angle)

        if not self.simulation_mode:
            self._set_servo_pulse(joint.channel, pulse_us)

        joint.current_angle_rad = clamped_angle
        return clamped_angle

    def set_all_angles(self, angles_rad):
        """Set all 4 joints simultaneously.

        Args:
            angles_rad: list of 4 floats [joint1, joint2, joint3, joint4] in radians

        Returns:
            list of actual angles after clamping
        """
        if len(angles_rad) < 4:
            print(f"[ServoDriver] Expected 4 angles, got {len(angles_rad)}")
            return None

        actual_angles = []
        for i, joint_name in enumerate(self.joint_order):
            actual = self.set_angle(joint_name, angles_rad[i])
            actual_angles.append(actual if actual is not None else 0.0)

        return actual_angles

    def get_angles(self):
        """Get current joint angles (radians)."""
        return [self.joints[name].current_angle_rad for name in self.joint_order]

    def get_joint_limits(self):
        """Get joint limits as dict of (min, max) tuples."""
        return {
            name: (joint.min_angle_rad, joint.max_angle_rad)
            for name, joint in self.joints.items()
        }

    def go_home(self):
        """Move all joints to their default positions."""
        defaults = [self.joints[name].default_position_rad for name in self.joint_order]
        return self.set_all_angles(defaults)

    def disable_all(self):
        """Disable all servo outputs (let them go limp).

        Useful for manually positioning the arm for calibration.
        """
        if self.simulation_mode or self.pca is None:
            print("[ServoDriver] Servos disabled (simulation mode)")
            return

        for name in self.joint_order:
            channel = self.joints[name].channel
            try:
                self.pca.channels[channel].duty_cycle = 0
            except Exception as e:
                print(f"[ServoDriver] Error disabling channel {channel}: {e}")

        print("[ServoDriver] All servos disabled (limp)")

    def shutdown(self):
        """Safely shutdown the servo driver."""
        print("[ServoDriver] Shutting down...")

        # Move to home position first
        try:
            self.go_home()
            time.sleep(0.5)
        except Exception:
            pass

        # Deinitialize PCA9685
        if self.pca is not None:
            try:
                self.pca.deinit()
            except Exception:
                pass

        self.initialized = False
        print("[ServoDriver] ✓ Shutdown complete")

    def print_status(self):
        """Print current status of all joints."""
        print("\n" + "=" * 60)
        print("SERVO DRIVER STATUS")
        print("=" * 60)
        print(f"  Mode: {'SIMULATION' if self.simulation_mode else 'HARDWARE'}")
        print(f"  Initialized: {self.initialized}")
        print(f"  I2C Address: 0x{self.i2c_address:02X}")
        print(f"  PWM Frequency: {self.pwm_frequency} Hz")
        print()

        for name in self.joint_order:
            joint = self.joints[name]
            pulse = joint.angle_to_pulse(joint.current_angle_rad)
            print(f"  {name}:")
            print(f"    Channel: {joint.channel}")
            print(f"    Angle: {joint.current_angle_rad:.3f} rad "
                  f"({math.degrees(joint.current_angle_rad):.1f}°)")
            print(f"    Pulse: {pulse} µs")
            print(f"    Range: [{math.degrees(joint.min_angle_rad):.0f}°, "
                  f"{math.degrees(joint.max_angle_rad):.0f}°] "
                  f"(servo: {joint.servo_range_deg}°)")
            print(f"    Inverted: {joint.inverted}")
            print(f"    Offset: {math.degrees(joint.offset_rad):.1f}°")
        print("=" * 60 + "\n")
