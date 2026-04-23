#!/usr/bin/env python3
"""
Standalone Servo Test for Assem6 Robot Arm
No ROS2 required — tests PCA9685 servos directly.

Usage:
  python3 servo_test.py                # Interactive menu
  python3 servo_test.py --sweep        # Sweep all servos
  python3 servo_test.py --joint 1      # Test joint 1 only
"""

import sys
import os
import time
import math
import argparse

# Add parent directory to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from assem6_hardware.servo_driver import PCA9685ServoDriver


def sweep_test(driver, joint_name=None, speed=0.5):
    """Sweep servo(s) through their full range."""
    joints_to_test = [joint_name] if joint_name else driver.joint_order

    for name in joints_to_test:
        if name not in driver.joints:
            print(f"Unknown joint: {name}")
            continue

        joint = driver.joints[name]
        print(f"\n--- Sweeping {name} ---")
        print(f"  Range: {math.degrees(joint.min_angle_rad):.0f}° to "
              f"{math.degrees(joint.max_angle_rad):.0f}°")

        # Sweep from min to max
        steps = 50
        range_rad = joint.max_angle_rad - joint.min_angle_rad

        print(f"  Moving to minimum ({math.degrees(joint.min_angle_rad):.0f}°)...")
        for i in range(steps + 1):
            t = i / steps
            angle = joint.min_angle_rad * t  # Go from 0 to min
            driver.set_angle(name, joint.min_angle_rad * t + joint.current_angle_rad * (1 - t))
            time.sleep(speed / steps)

        driver.set_angle(name, joint.min_angle_rad)
        time.sleep(0.3)

        print(f"  Sweeping to maximum ({math.degrees(joint.max_angle_rad):.0f}°)...")
        for i in range(steps + 1):
            t = i / steps
            angle = joint.min_angle_rad + t * range_rad
            driver.set_angle(name, angle)
            time.sleep(speed * 2 / steps)

        time.sleep(0.3)

        print(f"  Returning to center (0°)...")
        for i in range(steps + 1):
            t = i / steps
            angle = joint.max_angle_rad * (1 - t)
            driver.set_angle(name, angle)
            time.sleep(speed / steps)

        driver.set_angle(name, 0.0)
        time.sleep(0.3)

        print(f"  ✓ {name} sweep complete")


def interactive_mode(driver):
    """Interactive command-line control of servos."""
    print("\n" + "=" * 50)
    print("SERVO TEST - INTERACTIVE MODE")
    print("=" * 50)
    print("Commands:")
    print("  j<N> <angle>  - Set joint N (1-4) to angle in degrees")
    print("  all <a1> <a2> <a3> <a4>  - Set all joints (degrees)")
    print("  sweep [N]     - Sweep test (all or joint N)")
    print("  home          - All joints to 0°")
    print("  limp          - Disable all servos")
    print("  status        - Show current positions")
    print("  quit          - Exit")
    print("=" * 50)

    while True:
        try:
            cmd = input("\nservo> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nExiting...")
            break

        if not cmd:
            continue

        parts = cmd.split()
        command = parts[0].lower()

        try:
            if command in ('quit', 'q', 'exit'):
                break

            elif command == 'home':
                driver.go_home()
                print("✓ All joints at home (0°)")

            elif command == 'limp':
                driver.disable_all()

            elif command == 'status':
                driver.print_status()

            elif command.startswith('sweep'):
                if len(parts) > 1:
                    joint_name = f"joint{parts[1]}"
                    sweep_test(driver, joint_name)
                else:
                    sweep_test(driver)

            elif command.startswith('j') and len(command) == 2 and command[1].isdigit():
                joint_num = int(command[1])
                if 1 <= joint_num <= 4 and len(parts) >= 2:
                    angle_deg = float(parts[1])
                    angle_rad = math.radians(angle_deg)
                    joint_name = f"joint{joint_num}"
                    actual = driver.set_angle(joint_name, angle_rad)
                    if actual is not None:
                        print(f"✓ {joint_name} → {math.degrees(actual):.1f}°")
                else:
                    print("Usage: j<1-4> <angle_degrees>")

            elif command == 'all' and len(parts) >= 5:
                angles_deg = [float(parts[i]) for i in range(1, 5)]
                angles_rad = [math.radians(a) for a in angles_deg]
                actual = driver.set_all_angles(angles_rad)
                if actual:
                    print(f"✓ Joints → [{', '.join(f'{math.degrees(a):.1f}°' for a in actual)}]")

            else:
                print(f"Unknown command: '{command}'. Type 'help' for commands.")

        except ValueError as e:
            print(f"Error: invalid number - {e}")
        except Exception as e:
            print(f"Error: {e}")

    driver.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Assem6 Servo Test Tool')
    parser.add_argument('--sweep', action='store_true', help='Run sweep test on all servos')
    parser.add_argument('--joint', type=int, choices=[1, 2, 3, 4], help='Test specific joint only')
    parser.add_argument('--config', type=str, default=None, help='Path to servo_config.yaml')
    args = parser.parse_args()

    # Find config file
    config_path = args.config
    if not config_path:
        # Try common locations
        for path in [
            os.path.join(os.path.dirname(__file__), '..', 'config', 'servo_config.yaml'),
            os.path.expanduser('~/x-bot/src/assem6_hardware/config/servo_config.yaml'),
        ]:
            if os.path.exists(path):
                config_path = path
                break

    print("=" * 50)
    print("ASSEM6 SERVO TEST")
    print("=" * 50)

    driver = PCA9685ServoDriver(config_path)
    driver.initialize()
    driver.print_status()

    if args.sweep:
        joint_name = f"joint{args.joint}" if args.joint else None
        sweep_test(driver, joint_name)
        driver.shutdown()
    elif args.joint:
        sweep_test(driver, f"joint{args.joint}")
        driver.shutdown()
    else:
        interactive_mode(driver)


if __name__ == '__main__':
    main()
