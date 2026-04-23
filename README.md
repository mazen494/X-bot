# X-bot — Assem6 4-DOF Barista Robot Arm

A ROS2-based **4-DOF barista robot arm** that autonomously prepares drink orders. The system includes full Gazebo simulation, RViz visualization, MoveIt2 motion planning, and real hardware control via a PCA9685 servo driver on a Raspberry Pi 4.

---

## Features

- **8 Barista Stations**: Home, Cup Dispenser, Ice Dispenser, 4 Drink Dispensers, Service Point
- **Touchscreen GUI**: Order drinks with a sleek Tkinter interface
- **Smooth Motion**: S-curve and ease-in-out interpolation for natural movements
- **Simulation + Hardware**: Run in Gazebo or deploy to real servos — same code
- **Standalone Mode**: Run on Raspberry Pi without ROS2 using `barista_standalone.py`
- **Inverse Kinematics**: Numerical IK solver for Cartesian path planning
- **MoveIt2 Integration**: Full MoveIt2 config for advanced motion planning

---

## Hardware

| Component | Specification |
|-----------|---------------|
| **Controller** | Raspberry Pi 4 (4GB RAM) |
| **Servo Driver** | PCA9685 16-Channel I2C PWM Board |
| **Joint 1** | 80 kg·cm, 270° servo → CH15 |
| **Joint 2** | 150 kg·cm, 180° servo → CH14 |
| **Joint 3** | 80 kg·cm, 180° servo → CH13 |
| **Joint 4** | 40 kg·cm, 180° servo → CH10 |
| **Power** | 5–6V high-current supply (separate from Pi) |

> ⚠️ See [PI_SETUP.md](PI_SETUP.md) for full wiring diagram and deployment guide.

---

## Repository Structure

```
x-bot/
├── PI_SETUP.md                      # Raspberry Pi deployment guide
├── README.md                        # This file
└── src/
    ├── assem6/                      # Main simulation package
    │   ├── scripts/                 # Python nodes (GUI, IK, path planning)
    │   ├── urdf/                    # Robot URDF models
    │   ├── meshes/                  # 3D mesh files (STL)
    │   ├── config/                  # RViz configs, recipes, controllers
    │   ├── launch/                  # ROS2 launch files
    │   └── worlds/                  # Gazebo world files
    │
    ├── assem6_hardware/             # Hardware control package
    │   ├── assem6_hardware/         # PCA9685 driver, servo bridge, standalone app
    │   ├── config/                  # Servo channel mapping & calibration
    │   ├── launch/                  # Hardware launch file
    │   └── scripts/                 # Pi setup script
    │
    └── assem6_moveit_config/        # MoveIt2 motion planning config
        ├── config/                  # SRDF, joint limits, kinematics
        ├── launch/                  # MoveIt launch files
        └── scripts/                # Scenario runner
```

---

## Quick Start

### Simulation (Ubuntu PC)

```bash
# Build
cd ~/x-bot
colcon build
source install/setup.bash

# Visualize in RViz
ros2 launch assem6 display.launch.py

# Run barista GUI (controls robot in RViz)
ros2 launch assem6 barista.launch.py

# Gazebo simulation
ros2 launch assem6 gazebo.launch.py
```

### Hardware (Raspberry Pi 4)

```bash
# 1. Setup Pi (one-time)
chmod +x src/assem6_hardware/scripts/setup_pi.sh
./src/assem6_hardware/scripts/setup_pi.sh
sudo reboot

# 2. Test servos
python3 src/assem6_hardware/assem6_hardware/servo_test.py

# 3. Calibrate
python3 src/assem6_hardware/assem6_hardware/calibrate_servos.py

# 4a. Standalone mode (no ROS2 needed)
python3 src/assem6_hardware/assem6_hardware/barista_standalone.py

# 4b. ROS2 bridge mode (connect to PC)
source install/setup.bash
ros2 launch assem6_hardware hardware.launch.py
```

---

## Key Files

| File | Description |
|------|-------------|
| `servo_driver.py` | PCA9685 driver — converts joint angles to PWM signals |
| `servo_bridge_node.py` | ROS2 bridge — mirrors simulation joint states to hardware |
| `barista_standalone.py` | Standalone barista GUI + servo control (no ROS2) |
| `barista_gui.py` | ROS2 barista GUI — order drinks, drives simulation |
| `servo_config.yaml` | Channel mapping, pulse ranges, joint limits |
| `Assem6.urdf` | Robot model for simulation |
| `Assem6_hardware.urdf` | Robot model for hardware (no Gazebo plugins) |

---

## Architecture

```
┌─────────────────────────────────────────────────┐
│               RASPBERRY PI 4                     │
│                                                  │
│  ┌──────────────┐    ┌──────────────┐           │
│  │ barista_gui   │    │ robot_state  │           │
│  │ (touchscreen) │    │ _publisher   │           │
│  └──────┬───────┘    └──────────────┘           │
│         │ /joint_states                          │
│         ▼                                        │
│  ┌──────────────┐    ┌──────────────┐           │
│  │ servo_bridge  │───▶│ PCA9685      │──▶ Servos│
│  │ node          │    │ Driver       │           │
│  └──────┬───────┘    └──────────────┘           │
│         │ /servo_joint_states                    │
└─────────┼────────────────────────────────────────┘
          │ ROS2 DDS (WiFi/Ethernet)
┌─────────┼────────────────────────────────────────┐
│         ▼          UBUNTU PC (optional)          │
│  ┌──────────────┐    ┌──────────────┐           │
│  │ RViz2        │    │ MoveIt2      │           │
│  │ (visualize)  │    │ (planning)   │           │
│  └──────────────┘    └──────────────┘           │
└──────────────────────────────────────────────────┘
```

---

## License

This project was developed as a university robotics project.
