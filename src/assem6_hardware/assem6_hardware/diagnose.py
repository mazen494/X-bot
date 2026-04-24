#!/usr/bin/env python3
"""
Quick diagnostic for Assem6 servo hardware.
Run on the Pi: python3 diagnose.py
"""
import sys
import os

print("=" * 55)
print("  ASSEM6 HARDWARE DIAGNOSTIC")
print("=" * 55)

# 1. Check I2C device
print("\n[1] Checking I2C bus...")
try:
    import subprocess
    result = subprocess.run(['i2cdetect', '-y', '1'], capture_output=True, text=True)
    print(result.stdout)
    if '40' in result.stdout:
        print("  ✓ PCA9685 detected at 0x40")
    else:
        print("  ✗ PCA9685 NOT detected at 0x40!")
        print("    - Check wiring: SDA→GPIO2 (pin3), SCL→GPIO3 (pin5)")
        print("    - Check power: VCC→3.3V (pin1), GND→GND (pin6)")
        print("    - Run: sudo raspi-config → Interface → I2C → Enable")
except FileNotFoundError:
    print("  ✗ i2cdetect not found. Install: sudo apt install i2c-tools")

# 2. Check Python libraries
print("\n[2] Checking Python libraries...")
libs = {
    'board': 'adafruit-blinka',
    'busio': 'adafruit-blinka',
    'adafruit_pca9685': 'adafruit-circuitpython-pca9685',
    'adafruit_motor': 'adafruit-circuitpython-motor',
}
all_ok = True
for mod, pkg in libs.items():
    try:
        __import__(mod)
        print(f"  ✓ {mod}")
    except ImportError:
        print(f"  ✗ {mod} — pip3 install {pkg}")
        all_ok = False

if not all_ok:
    print("\n  Install all missing libs:")
    print("  pip3 install adafruit-blinka adafruit-circuitpython-pca9685 adafruit-circuitpython-motor")

# 3. Check tkinter
print("\n[3] Checking tkinter...")
try:
    import tkinter
    print("  ✓ tkinter available")
except ImportError:
    print("  ✗ tkinter missing — sudo apt install python3-tk")

# 4. Try to initialize PCA9685
print("\n[4] Attempting PCA9685 initialization...")
try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685

    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c, address=0x40)
    pca.frequency = 50
    print("  ✓ PCA9685 initialized successfully!")

    # 5. Try moving a servo briefly
    print("\n[5] Testing servo on channel 15 (joint1)...")
    print("    Sending 1500µs pulse (center position)...")
    period_us = 20000.0  # 50Hz
    pulse_us = 1500
    duty = int((pulse_us / period_us) * 65535)
    pca.channels[15].duty_cycle = duty

    import time
    time.sleep(1)
    print("  ✓ Pulse sent! Did the servo on channel 15 move?")

    # Disable
    pca.channels[15].duty_cycle = 0
    pca.deinit()
    print("  ✓ PCA9685 deinitialized")

except Exception as e:
    print(f"  ✗ Error: {e}")
    import traceback
    traceback.print_exc()

# 6. Check config file
print("\n[6] Checking servo_config.yaml...")
config_paths = [
    os.path.join(os.path.dirname(__file__), '..', 'config', 'servo_config.yaml'),
    os.path.expanduser('~/x-bot/src/assem6_hardware/config/servo_config.yaml'),
]
found = False
for p in config_paths:
    if os.path.exists(p):
        print(f"  ✓ Found config: {os.path.abspath(p)}")
        found = True
        break
if not found:
    print("  ✗ servo_config.yaml not found!")

# 7. Check station angles vs joint limits
print("\n[7] Checking station angles vs joint limits...")
joint_limits = {
    'joint1': (-2.356, 2.356),   # 270° servo
    'joint2': (-1.5708, 1.5708), # 180° servo
    'joint3': (-1.5708, 1.5708), # 180° servo
    'joint4': (-1.5708, 1.5708), # 180° servo
}
stations = {
    'home':           [1.26, -0.44, 1.57, -0.43],
    'cup_dispenser':  [1.58, -0.89, 1.57, -0.87],
    'ice_dispenser':  [-0.65, -0.89, 1.57, -0.87],
    'drink1':         [0.79, -0.89, 1.57, -0.87],
    'drink2':         [0.00, -0.89, 1.57, -0.87],
    'drink3':         [2.45, -0.89, 1.57, -0.87],
    'drink4':         [-3.08, -0.89, 1.57, -0.87],
    'service_point':  [-1.54, -0.89, 1.57, -0.87],
}
import math
issues = []
for station_name, angles in stations.items():
    for i, (angle, (jname, (lo, hi))) in enumerate(zip(angles, joint_limits.items())):
        if angle < lo or angle > hi:
            issues.append(f"  ✗ {station_name}.{jname}: {math.degrees(angle):.1f}° "
                          f"is OUTSIDE [{math.degrees(lo):.0f}°, {math.degrees(hi):.0f}°]")
if issues:
    print("  PROBLEMS FOUND:")
    for issue in issues:
        print(issue)
else:
    print("  ✓ All station angles within joint limits")

print("\n" + "=" * 55)
print("  DIAGNOSTIC COMPLETE")
print("=" * 55)
