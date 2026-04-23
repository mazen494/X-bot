#!/bin/bash
# =============================================================
# Raspberry Pi 4 Setup Script for Assem6 Robot Arm
# =============================================================
#
# This script installs ROS2 Humble and all dependencies needed
# to run the Assem6 hardware interface on a Raspberry Pi 4.
#
# Requirements:
#   - Raspberry Pi 4 (4GB)
#   - Ubuntu 22.04 Server/Desktop (64-bit ARM)
#   - Internet connection
#
# Usage:
#   chmod +x setup_pi.sh
#   ./setup_pi.sh
#
# =============================================================

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}"
echo "============================================================="
echo "  ASSEM6 ROBOT ARM - RASPBERRY PI 4 SETUP"
echo "============================================================="
echo -e "${NC}"

# ─────────────────────────────────────────────────────────────
# Step 0: Check system
# ─────────────────────────────────────────────────────────────
echo -e "${YELLOW}[Step 0] Checking system...${NC}"

# Check if running on ARM (Raspberry Pi)
ARCH=$(uname -m)
echo "  Architecture: $ARCH"

# Check Ubuntu version
if [ -f /etc/os-release ]; then
    . /etc/os-release
    echo "  OS: $PRETTY_NAME"
    if [ "$VERSION_ID" != "22.04" ]; then
        echo -e "${YELLOW}  WARNING: This script is designed for Ubuntu 22.04.${NC}"
        echo -e "${YELLOW}  You are running $VERSION_ID. Proceed with caution.${NC}"
        read -p "  Continue? (y/n): " -r
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
fi

# Check available memory
MEM_TOTAL=$(free -m | awk '/^Mem:/{print $2}')
echo "  RAM: ${MEM_TOTAL} MB"

if [ "$MEM_TOTAL" -lt 3000 ]; then
    echo -e "${YELLOW}  WARNING: Less than 3GB RAM detected.${NC}"
    echo -e "${YELLOW}  Building may require swap space.${NC}"
fi

# ─────────────────────────────────────────────────────────────
# Step 1: System updates
# ─────────────────────────────────────────────────────────────
echo -e "\n${YELLOW}[Step 1] Updating system packages...${NC}"
sudo apt update && sudo apt upgrade -y

# ─────────────────────────────────────────────────────────────
# Step 2: Enable I2C
# ─────────────────────────────────────────────────────────────
echo -e "\n${YELLOW}[Step 2] Enabling I2C...${NC}"

# Enable I2C in config
if [ -f /boot/firmware/config.txt ]; then
    CONFIG_FILE="/boot/firmware/config.txt"
elif [ -f /boot/config.txt ]; then
    CONFIG_FILE="/boot/config.txt"
else
    echo -e "${RED}  WARNING: Cannot find boot config file${NC}"
    CONFIG_FILE=""
fi

if [ -n "$CONFIG_FILE" ]; then
    if ! grep -q "^dtparam=i2c_arm=on" "$CONFIG_FILE"; then
        echo "dtparam=i2c_arm=on" | sudo tee -a "$CONFIG_FILE"
        echo "  ✓ I2C enabled in $CONFIG_FILE"
    else
        echo "  ✓ I2C already enabled"
    fi
fi

# Load I2C kernel module
sudo modprobe i2c-dev 2>/dev/null || true
if ! grep -q "i2c-dev" /etc/modules; then
    echo "i2c-dev" | sudo tee -a /etc/modules
fi

# Install I2C tools
sudo apt install -y i2c-tools python3-smbus
echo "  ✓ I2C tools installed"

# Add user to i2c group
sudo usermod -aG i2c $USER 2>/dev/null || true
echo "  ✓ User added to i2c group"

# ─────────────────────────────────────────────────────────────
# Step 3: Install ROS2 Humble
# ─────────────────────────────────────────────────────────────
echo -e "\n${YELLOW}[Step 3] Installing ROS2 Humble...${NC}"

# Check if ROS2 is already installed
if [ -f /opt/ros/humble/setup.bash ]; then
    echo "  ✓ ROS2 Humble already installed"
else
    # Set locale
    sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    # Add ROS2 apt repository
    sudo apt install -y software-properties-common curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
        sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update

    # Install ROS2 Humble (base only - no GUI packages to save space on Pi)
    echo "  Installing ROS2 Humble Base (this will take a while)..."
    sudo apt install -y ros-humble-ros-base

    # Install additional ROS2 packages needed
    sudo apt install -y \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-sensor-msgs \
        ros-humble-trajectory-msgs \
        python3-colcon-common-extensions \
        python3-rosdep

    # Initialize rosdep
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    fi
    rosdep update

    echo -e "${GREEN}  ✓ ROS2 Humble installed!${NC}"
fi

# ─────────────────────────────────────────────────────────────
# Step 4: Install Python dependencies
# ─────────────────────────────────────────────────────────────
echo -e "\n${YELLOW}[Step 4] Installing Python dependencies...${NC}"

# Ensure pip is installed
sudo apt install -y python3-pip python3-venv

# Install PCA9685 servo driver library
pip3 install --user \
    adafruit-circuitpython-pca9685 \
    adafruit-circuitpython-motor \
    adafruit-circuitpython-servokit \
    RPi.GPIO \
    pyyaml

echo "  ✓ Python dependencies installed"

# ─────────────────────────────────────────────────────────────
# Step 5: Install tkinter for GUI on touchscreen
# ─────────────────────────────────────────────────────────────
echo -e "\n${YELLOW}[Step 5] Installing GUI dependencies...${NC}"
sudo apt install -y python3-tk
echo "  ✓ tkinter installed"

# ─────────────────────────────────────────────────────────────
# Step 6: Setup ROS2 workspace
# ─────────────────────────────────────────────────────────────
echo -e "\n${YELLOW}[Step 6] Setting up ROS2 workspace...${NC}"

# Add ROS2 to bashrc if not already there
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS2 Humble" >> ~/.bashrc
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "  ✓ ROS2 added to .bashrc"
fi

# Set ROS_DOMAIN_ID for multi-machine communication
if ! grep -q "ROS_DOMAIN_ID" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS2 multi-machine communication" >> ~/.bashrc
    echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
    echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
    echo "  ✓ ROS_DOMAIN_ID set to 42"
fi

# Source ROS2 for the current session
source /opt/ros/humble/setup.bash

# ─────────────────────────────────────────────────────────────
# Step 7: Build the workspace
# ─────────────────────────────────────────────────────────────
echo -e "\n${YELLOW}[Step 7] Building ROS2 workspace...${NC}"

# Determine workspace path
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Walk up from src/assem6_hardware/scripts/ to workspace root
WS_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

if [ ! -f "$WS_ROOT/src/assem6_hardware/package.xml" ]; then
    echo -e "${RED}  ERROR: Cannot find workspace at $WS_ROOT${NC}"
    echo "  Please run from within the x-bot workspace"
    exit 1
fi

echo "  Workspace: $WS_ROOT"
cd "$WS_ROOT"

# Build only the packages we need (skip Gazebo/MoveIt to save time on Pi)
colcon build --packages-select assem6 assem6_hardware --symlink-install

# Source the workspace
if ! grep -q "source.*x-bot/install/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# Assem6 workspace" >> ~/.bashrc
    echo "source $WS_ROOT/install/setup.bash" >> ~/.bashrc
fi

source "$WS_ROOT/install/setup.bash"

echo -e "${GREEN}  ✓ Workspace built successfully!${NC}"

# ─────────────────────────────────────────────────────────────
# Step 8: Increase swap (for building on 4GB Pi)
# ─────────────────────────────────────────────────────────────
echo -e "\n${YELLOW}[Step 8] Checking swap space...${NC}"

SWAP_TOTAL=$(free -m | awk '/^Swap:/{print $2}')
if [ "$SWAP_TOTAL" -lt 1024 ]; then
    echo "  Current swap: ${SWAP_TOTAL}MB (increasing to 2GB)..."
    sudo fallocate -l 2G /swapfile 2>/dev/null || sudo dd if=/dev/zero of=/swapfile bs=1M count=2048
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
    sudo swapon /swapfile
    if ! grep -q "/swapfile" /etc/fstab; then
        echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
    fi
    echo "  ✓ Swap increased to 2GB"
else
    echo "  ✓ Swap OK (${SWAP_TOTAL}MB)"
fi

# ─────────────────────────────────────────────────────────────
# Step 9: Test I2C connectivity
# ─────────────────────────────────────────────────────────────
echo -e "\n${YELLOW}[Step 9] Testing I2C...${NC}"

if command -v i2cdetect &> /dev/null; then
    echo "  Scanning I2C bus 1:"
    sudo i2cdetect -y 1 2>/dev/null || echo "  (I2C scan failed - may need reboot)"
    echo ""
    echo "  If you see '40' in the grid above, PCA9685 is detected!"
else
    echo "  i2cdetect not available - install with: sudo apt install i2c-tools"
fi

# ─────────────────────────────────────────────────────────────
# Done!
# ─────────────────────────────────────────────────────────────
echo -e "\n${GREEN}"
echo "============================================================="
echo "  ✓ SETUP COMPLETE!"
echo "============================================================="
echo -e "${NC}"
echo "Next steps:"
echo ""
echo "  1. REBOOT the Pi (required for I2C changes):"
echo "     sudo reboot"
echo ""
echo "  2. After reboot, test I2C:"
echo "     sudo i2cdetect -y 1"
echo "     (You should see '40' for PCA9685)"
echo ""
echo "  3. Test servos without ROS2:"
echo "     cd $WS_ROOT"
echo "     python3 src/assem6_hardware/assem6_hardware/servo_test.py"
echo ""
echo "  4. Calibrate servos:"
echo "     python3 src/assem6_hardware/assem6_hardware/calibrate_servos.py"
echo ""
echo "  5. Launch the robot:"
echo "     ros2 launch assem6_hardware hardware.launch.py"
echo ""
echo "  NOTE: Make sure your Ubuntu PC uses the same ROS_DOMAIN_ID:"
echo "     export ROS_DOMAIN_ID=42"
echo ""
