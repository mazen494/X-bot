# Assem6 Robot Arm — Raspberry Pi 4 Hardware Deployment

Complete guide to deploying the Assem6 barista robot arm on a Raspberry Pi 4 with real servo motors.

---

## Hardware Requirements

| Component | Specification |
|-----------|--------------|
| **Raspberry Pi** | Pi 4 (4GB RAM) |
| **Servo Driver** | PCA9685 16-Channel I2C PWM Board |
| **Joint 1 Servo** | 80 kg·cm torque, 270° range |
| **Joint 2 Servo** | 150 kg·cm torque, 180° range |
| **Joint 3 Servo** | 80 kg·cm torque, 180° range |
| **Joint 4 Servo** | 40 kg·cm torque, 180° range |
| **Power Supply** | 5–6V high-current supply for servos (separate from Pi!) |
| **Display** | Touchscreen (for barista GUI) |
| **OS** | Ubuntu 22.04 Server/Desktop (64-bit ARM) |

> ⚠️ **IMPORTANT**: Never power the servos from the Pi's 5V pin! Use a separate power supply (5–6V, at least 10A) connected to the PCA9685's V+ terminal.

---

## Wiring Diagram

```
Raspberry Pi 4              PCA9685 Board              Servos
┌──────────┐               ┌─────────────┐
│ GPIO 2   │──── SDA ─────│ SDA         │
│ (Pin 3)  │               │             │
│ GPIO 3   │──── SCL ─────│ SCL         │           ┌──────────┐
│ (Pin 5)  │               │             │  CH0 ────│ Joint 1  │ (270°, 80kg.cm)
│          │               │ GND     V+  │  CH1 ────│ Joint 2  │ (180°, 150kg.cm)
│ GND      │──── GND ─────│  │       │  │  CH2 ────│ Joint 3  │ (180°, 80kg.cm)
│ (Pin 6)  │               │  │       │  │  CH3 ────│ Joint 4  │ (180°, 40kg.cm)
└──────────┘               └──┼───────┼──┘           └──────────┘
                               │       │
                          ┌────┘       └────┐
                          │    POWER        │
                          │    SUPPLY       │
                          │  GND     5-6V   │
                          └─────────────────┘
```

### Pin Connections

| Pi Pin | Pi GPIO | PCA9685 Pin | Purpose |
|--------|---------|-------------|---------|
| Pin 3  | GPIO 2  | SDA         | I2C Data |
| Pin 5  | GPIO 3  | SCL         | I2C Clock |
| Pin 6  | GND     | GND         | Common Ground |

### Servo Channel Mapping

| PCA9685 Channel | Joint | Servo Spec | Angle Range |
|-----------------|-------|------------|-------------|
| CH0 | joint1 (base rotation) | 80 kg·cm, 270° | ±135° (±2.356 rad) |
| CH1 | joint2 (shoulder) | 150 kg·cm, 180° | ±90° (±1.571 rad) |
| CH2 | joint3 (elbow) | 80 kg·cm, 180° | ±90° (±1.571 rad) |
| CH3 | joint4 (wrist) | 40 kg·cm, 180° | ±90° (±1.571 rad) |

---

## Step 1: Install Ubuntu on Pi

1. Download **Ubuntu 22.04.x Server (64-bit)** for Raspberry Pi from:
   https://ubuntu.com/download/raspberry-pi

2. Flash to SD card using **Raspberry Pi Imager** or **balenaEtcher**

3. Boot the Pi and complete initial setup (username, WiFi, etc.)

4. If using a touchscreen, install desktop environment:
   ```bash
   sudo apt install ubuntu-desktop-minimal
   ```

---

## Step 2: Run the Setup Script

Clone or copy the `x-bot` workspace to the Pi, then:

```bash
cd ~/x-bot
chmod +x src/assem6_hardware/scripts/setup_pi.sh
./src/assem6_hardware/scripts/setup_pi.sh
```

This script will:
- Enable I2C
- Install ROS2 Humble
- Install PCA9685 Python libraries
- Install tkinter for the GUI
- Build the workspace
- Setup swap space
- Configure ROS_DOMAIN_ID

**After the script completes, REBOOT:**
```bash
sudo reboot
```

---

## Step 3: Verify I2C Connection

After reboot, verify the PCA9685 is detected:

```bash
sudo i2cdetect -y 1
```

Expected output (the `40` indicates PCA9685 at address 0x40):
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- --
```

If you don't see `40`, check:
- SDA/SCL wiring
- PCA9685 power (it needs 3.3V–5V on VCC, separate from V+)
- I2C enabled: `sudo raspi-config` → Interface Options → I2C

---

## Step 4: Test Servos (No ROS2 Required)

Test that servos move correctly before involving ROS2:

```bash
cd ~/x-bot
python3 src/assem6_hardware/assem6_hardware/servo_test.py
```

In the interactive mode:
```
servo> j1 0       # Joint 1 to 0°
servo> j1 45      # Joint 1 to 45°
servo> j2 -30     # Joint 2 to -30°
servo> sweep       # Sweep all servos
servo> sweep 1     # Sweep joint 1 only
servo> home        # All to 0°
```

---

## Step 5: Calibrate Servos

Run the calibration wizard to find correct PWM values:

```bash
python3 src/assem6_hardware/assem6_hardware/calibrate_servos.py
```

The wizard will:
1. Ask you to position each joint at its **center** (0°)
2. Then at its **minimum** angle
3. Then at its **maximum** angle
4. Save the correct pulse widths to `servo_config.yaml`

After calibration, copy the generated config:
```bash
cp servo_config_calibrated.yaml src/assem6_hardware/config/servo_config.yaml
```

---

## Step 6: Launch the Robot

### On the Raspberry Pi:

```bash
source ~/x-bot/install/setup.bash
ros2 launch assem6_hardware hardware.launch.py
```

This starts:
- **robot_state_publisher** (publishes robot TF tree)
- **servo_bridge** (bridges joint commands → PCA9685 servos)
- **barista_gui** (touchscreen GUI for ordering drinks)

### On your Ubuntu PC (optional — for RViz visualization):

```bash
# Set the same domain ID as the Pi
export ROS_DOMAIN_ID=42

# Launch RViz
source ~/x-bot/install/setup.bash
ros2 launch assem6 display.launch.py
```

---

## Step 7: Multi-Machine ROS2 Setup

Both the Pi and your Ubuntu PC need to be on the **same network** and use the **same ROS_DOMAIN_ID**.

### On both machines, add to `~/.bashrc`:
```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

### Verify connectivity:
```bash
# On the Pi:
ros2 topic list

# On the PC (should see the same topics):
ros2 topic list
```

### Check servo feedback:
```bash
# From the PC, watch Pi's servo positions:
ros2 topic echo /servo_joint_states
```

---

## Architecture

```
┌─────────────────────────────────────────────────────┐
│                  RASPBERRY PI 4                      │
│                                                      │
│  ┌──────────────┐    ┌──────────────┐               │
│  │ barista_gui   │    │ robot_state  │               │
│  │ (touchscreen) │    │ _publisher   │               │
│  └──────┬───────┘    └──────────────┘               │
│         │ /joint_states                              │
│         ▼                                            │
│  ┌──────────────┐    ┌──────────────┐               │
│  │ servo_bridge  │───▶│ PCA9685      │──▶ Servos    │
│  │ node          │    │ Driver       │               │
│  └──────┬───────┘    └──────────────┘               │
│         │ /servo_joint_states                        │
└─────────┼────────────────────────────────────────────┘
          │ ROS2 DDS (WiFi/Ethernet)
┌─────────┼────────────────────────────────────────────┐
│         ▼          UBUNTU PC (optional)              │
│  ┌──────────────┐    ┌──────────────┐               │
│  │ RViz2        │    │ MoveIt2      │               │
│  │ (visualize)  │    │ (planning)   │               │
│  └──────────────┘    └──────────────┘               │
└──────────────────────────────────────────────────────┘
```

---

## Troubleshooting

### Servo doesn't move
1. Check power supply to PCA9685 V+ terminal
2. Check servo wiring (signal on correct channel)
3. Run `servo_test.py` to test individual servos
4. Check I2C: `sudo i2cdetect -y 1`

### Servo jitters
1. Use a dedicated power supply for servos (not Pi's 5V)
2. Add a large capacitor (1000µF) across the power rails
3. Reduce `update_rate` in servo_config.yaml

### ROS2 topics not visible between machines
1. Check both machines use `ROS_DOMAIN_ID=42`
2. Check `ROS_LOCALHOST_ONLY=0` (not 1)
3. Check firewall: `sudo ufw allow 7400:7500/udp`
4. Try: `ros2 multicast receive` on both machines

### GUI doesn't appear on touchscreen
1. Make sure you have a desktop environment installed
2. Set display: `export DISPLAY=:0`
3. Check tkinter: `python3 -c "import tkinter"`

### Build fails on Pi (out of memory)
1. Increase swap: `sudo fallocate -l 4G /swapfile`
2. Build with fewer parallel jobs: `colcon build --parallel-workers 1`
3. Build only needed packages: `colcon build --packages-select assem6 assem6_hardware`

---

## Config Reference

### servo_config.yaml

```yaml
servos:
  joint1:
    channel: 0           # PCA9685 channel
    min_pulse: 500        # µs at min angle
    max_pulse: 2500       # µs at max angle
    min_angle: -2.356     # radians
    max_angle: 2.356      # radians
    offset: 0.0           # calibration offset
    inverted: false       # reverse direction
    servo_range_deg: 270  # physical servo range
```

### Launch Arguments

```bash
# Full launch with GUI
ros2 launch assem6_hardware hardware.launch.py

# Headless (no GUI)
ros2 launch assem6_hardware hardware.launch.py gui:=false

# Custom config file
ros2 launch assem6_hardware hardware.launch.py config:=/path/to/config.yaml
```
