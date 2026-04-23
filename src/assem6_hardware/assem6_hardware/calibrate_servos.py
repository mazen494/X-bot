#!/usr/bin/env python3
"""
Servo Calibration Tool for Assem6 Robot Arm

Interactive tool to find the correct pulse width values for each servo
at known physical positions (e.g., 0°, max, min). Saves calibration
data to servo_config.yaml.

No ROS2 required.

Usage:
  python3 calibrate_servos.py
  python3 calibrate_servos.py --config /path/to/servo_config.yaml
"""

import sys
import os
import math
import time
import yaml
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from assem6_hardware.servo_driver import PCA9685ServoDriver


class ServoCalibrator:
    """Interactive servo calibration wizard."""

    def __init__(self, config_path=None):
        self.config_path = config_path
        self.driver = PCA9685ServoDriver(config_path)
        self.calibration_data = {}

    def initialize(self):
        """Initialize hardware."""
        self.driver.initialize()
        if self.driver.simulation_mode:
            print("\n⚠ SIMULATION MODE: Calibration values won't be real.")
            print("  Run this on the Raspberry Pi with servos connected.\n")
        return True

    def calibrate_joint(self, joint_name):
        """Interactive calibration for a single joint."""
        if joint_name not in self.driver.joints:
            print(f"Unknown joint: {joint_name}")
            return

        joint = self.driver.joints[joint_name]
        print(f"\n{'=' * 50}")
        print(f"CALIBRATING: {joint_name.upper()}")
        print(f"  Servo range: {joint.servo_range_deg}°")
        print(f"  Channel: {joint.channel}")
        print(f"{'=' * 50}")

        # Step 1: Find center position
        print(f"\n─── Step 1: CENTER POSITION ───")
        print(f"Adjust pulse until the joint is at its physical CENTER (0°).")
        print(f"Use +/- to adjust, enter to confirm.")

        center_pulse = (joint.min_pulse_us + joint.max_pulse_us) // 2
        self._adjust_pulse(joint, center_pulse)
        center_pulse = self._get_current_pulse(joint)
        print(f"  ✓ Center pulse: {center_pulse} µs\n")

        # Step 2: Find minimum position
        print(f"─── Step 2: MINIMUM POSITION ───")
        print(f"Adjust pulse until the joint is at its MINIMUM angle.")
        print(f"  Expected: {math.degrees(joint.min_angle_rad):.0f}°")

        self._adjust_pulse(joint, joint.min_pulse_us)
        min_pulse = self._get_current_pulse(joint)
        print(f"  ✓ Min pulse: {min_pulse} µs\n")

        # Step 3: Find maximum position
        print(f"─── Step 3: MAXIMUM POSITION ───")
        print(f"Adjust pulse until the joint is at its MAXIMUM angle.")
        print(f"  Expected: {math.degrees(joint.max_angle_rad):.0f}°")

        self._adjust_pulse(joint, joint.max_pulse_us)
        max_pulse = self._get_current_pulse(joint)
        print(f"  ✓ Max pulse: {max_pulse} µs\n")

        # Step 4: Check direction
        print(f"─── Step 4: DIRECTION CHECK ───")
        print(f"Moving to minimum position...")
        self._send_pulse(joint, min_pulse)
        time.sleep(0.5)

        inverted = False
        resp = input("Is the joint at its MOST NEGATIVE angle? (y/n): ").strip().lower()
        if resp != 'y':
            inverted = True
            print("  → Servo direction will be INVERTED")

        # Save calibration
        self.calibration_data[joint_name] = {
            'channel': joint.channel,
            'min_pulse': min_pulse,
            'max_pulse': max_pulse,
            'min_angle': joint.min_angle_rad,
            'max_angle': joint.max_angle_rad,
            'offset': 0.0,
            'inverted': inverted,
            'default_position': 0.0,
            'servo_range_deg': joint.servo_range_deg,
        }

        # Return to center
        self._send_pulse(joint, center_pulse)
        print(f"\n✓ {joint_name} calibration complete!")
        print(f"  Min pulse: {min_pulse} µs")
        print(f"  Max pulse: {max_pulse} µs")
        print(f"  Inverted: {inverted}")

    def _adjust_pulse(self, joint, initial_pulse):
        """Interactive pulse adjustment loop."""
        pulse = initial_pulse
        step = 10

        self._send_pulse(joint, pulse)
        time.sleep(0.2)

        print(f"  Current: {pulse} µs  |  Step: {step} µs")
        print(f"  Commands: +/- (adjust), s10/s50/s100 (set step), enter (confirm)")

        while True:
            try:
                cmd = input(f"  [{pulse} µs] > ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if cmd == '' or cmd == 'ok':
                break
            elif cmd == '+':
                pulse += step
            elif cmd == '-':
                pulse -= step
            elif cmd.startswith('+') and cmd[1:].isdigit():
                pulse += int(cmd[1:])
            elif cmd.startswith('-') and cmd[1:].isdigit():
                pulse -= int(cmd[1:])
            elif cmd.startswith('s') and cmd[1:].isdigit():
                step = int(cmd[1:])
                print(f"  Step size: {step} µs")
                continue
            elif cmd.isdigit():
                pulse = int(cmd)
            else:
                print(f"  Unknown command: {cmd}")
                continue

            pulse = max(200, min(3000, pulse))  # Safety clamp
            self._send_pulse(joint, pulse)
            time.sleep(0.05)
            print(f"  → {pulse} µs")

        self._current_pulse = pulse

    def _send_pulse(self, joint, pulse_us):
        """Send a raw pulse to the servo."""
        if self.driver.simulation_mode or self.driver.pca is None:
            return

        period_us = 1_000_000.0 / self.driver.pwm_frequency
        duty_cycle = int((pulse_us / period_us) * 65535)
        duty_cycle = max(0, min(65535, duty_cycle))

        try:
            self.driver.pca.channels[joint.channel].duty_cycle = duty_cycle
        except Exception as e:
            print(f"  Error: {e}")

    def _get_current_pulse(self, joint):
        """Get the last set pulse value."""
        return getattr(self, '_current_pulse', 1500)

    def save_calibration(self, output_path=None):
        """Save calibration data to YAML file."""
        if not output_path:
            output_path = self.config_path or 'servo_config_calibrated.yaml'

        # Build full config
        config = {
            'servos': self.calibration_data,
            'hardware': {
                'driver': 'pca9685',
                'i2c_bus': self.driver.i2c_bus,
                'i2c_address': self.driver.i2c_address,
                'pwm_frequency': self.driver.pwm_frequency,
                'update_rate': self.driver.update_rate,
                'max_speed': self.driver.max_speed_rad_s,
            }
        }

        # Convert numpy/float types for YAML
        def clean_for_yaml(obj):
            if isinstance(obj, dict):
                return {k: clean_for_yaml(v) for k, v in obj.items()}
            elif isinstance(obj, (list, tuple)):
                return [clean_for_yaml(v) for v in obj]
            elif isinstance(obj, float):
                return round(obj, 4)
            return obj

        config = clean_for_yaml(config)

        with open(output_path, 'w') as f:
            yaml.dump(config, f, default_flow_style=False, sort_keys=False)

        print(f"\n✓ Calibration saved to: {output_path}")
        print(f"  Copy this file to: src/assem6_hardware/config/servo_config.yaml")

    def run_full_calibration(self):
        """Run calibration wizard for all joints."""
        print("\n" + "=" * 50)
        print("ASSEM6 SERVO CALIBRATION WIZARD")
        print("=" * 50)
        print("\nThis tool will help you find the correct PWM pulse")
        print("widths for each servo at its physical limits.")
        print("\nMake sure:")
        print("  1. The robot is powered and servos are connected")
        print("  2. The arm can move freely (no obstructions)")
        print("  3. PCA9685 is connected to I2C bus\n")

        input("Press Enter to begin calibration...")

        for joint_name in self.driver.joint_order:
            self.calibrate_joint(joint_name)
            print()

        # Save results
        print("\n" + "=" * 50)
        print("CALIBRATION COMPLETE")
        print("=" * 50)

        for name, data in self.calibration_data.items():
            print(f"  {name}: pulse=[{data['min_pulse']}, {data['max_pulse']}] µs, "
                  f"inverted={data['inverted']}")

        save = input("\nSave calibration to file? (y/n): ").strip().lower()
        if save == 'y':
            self.save_calibration()

    def shutdown(self):
        """Clean shutdown."""
        self.driver.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Assem6 Servo Calibration Tool')
    parser.add_argument('--config', type=str, default=None, help='Path to existing servo_config.yaml')
    parser.add_argument('--joint', type=int, choices=[1, 2, 3, 4], help='Calibrate specific joint only')
    parser.add_argument('--output', type=str, default=None, help='Output path for calibrated config')
    args = parser.parse_args()

    # Find config
    config_path = args.config
    if not config_path:
        for path in [
            os.path.join(os.path.dirname(__file__), '..', 'config', 'servo_config.yaml'),
            os.path.expanduser('~/x-bot/src/assem6_hardware/config/servo_config.yaml'),
        ]:
            if os.path.exists(path):
                config_path = path
                break

    calibrator = ServoCalibrator(config_path)
    calibrator.initialize()

    try:
        if args.joint:
            calibrator.calibrate_joint(f"joint{args.joint}")
            calibrator.save_calibration(args.output)
        else:
            calibrator.run_full_calibration()
    except KeyboardInterrupt:
        print("\n\nCalibration interrupted.")
    finally:
        calibrator.shutdown()


if __name__ == '__main__':
    main()
