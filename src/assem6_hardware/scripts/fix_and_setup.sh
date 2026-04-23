#!/bin/bash
# =============================================================
# FIX SCRIPT for Raspberry Pi (Debian Trixie)
# =============================================================
# Run this FIRST to fix locale issues and install dependencies.
# ROS2 is NOT needed on the Pi - we use standalone Python instead.
#
# Usage:
#   chmod +x fix_and_setup.sh
#   ./fix_and_setup.sh
# =============================================================

set -e

echo "============================================================="
echo "  ASSEM6 ROBOT ARM - Pi Fix & Setup (Debian Trixie)"
echo "============================================================="

# ─── Step 1: Fix locale ───────────────────────────────────────
echo ""
echo "[Step 1] Fixing locale..."

# Generate the missing locales
sudo locale-gen en_US.UTF-8 en_GB.UTF-8 ar_EG.UTF-8 2>/dev/null || true
sudo dpkg-reconfigure -f noninteractive locales 2>/dev/null || true

# Set a working locale
sudo update-locale LANG=en_GB.UTF-8 LC_ALL=en_GB.UTF-8 2>/dev/null || true

# Fix for current session
export LANG=en_GB.UTF-8
export LC_ALL=en_GB.UTF-8

echo "  ✓ Locale fixed"

# ─── Step 2: Enable I2C ──────────────────────────────────────
echo ""
echo "[Step 2] Enabling I2C..."

# Enable I2C via raspi-config (non-interactive)
sudo raspi-config nonint do_i2c 0 2>/dev/null || true

# Also add to config.txt if not there
CONFIG_FILE=""
if [ -f /boot/firmware/config.txt ]; then
    CONFIG_FILE="/boot/firmware/config.txt"
elif [ -f /boot/config.txt ]; then
    CONFIG_FILE="/boot/config.txt"
fi

if [ -n "$CONFIG_FILE" ]; then
    if ! grep -q "^dtparam=i2c_arm=on" "$CONFIG_FILE"; then
        echo "dtparam=i2c_arm=on" | sudo tee -a "$CONFIG_FILE"
    fi
    echo "  ✓ I2C enabled in $CONFIG_FILE"
fi

# Load module
sudo modprobe i2c-dev 2>/dev/null || true
if ! grep -q "i2c-dev" /etc/modules 2>/dev/null; then
    echo "i2c-dev" | sudo tee -a /etc/modules
fi

# Add user to i2c group
sudo usermod -aG i2c $USER 2>/dev/null || true

# Install I2C tools
sudo apt-get install -y i2c-tools python3-smbus 2>/dev/null
echo "  ✓ I2C configured"

# ─── Step 3: Install Python dependencies ─────────────────────
echo ""
echo "[Step 3] Installing Python dependencies..."

sudo apt-get install -y \
    python3-pip \
    python3-venv \
    python3-tk \
    python3-yaml \
    python3-dev \
    python3-smbus2 2>/dev/null || true

# Install PCA9685 servo driver libraries
pip3 install --user --break-system-packages \
    adafruit-circuitpython-pca9685 \
    adafruit-circuitpython-motor \
    adafruit-circuitpython-servokit \
    pyyaml 2>/dev/null || \
pip3 install --user \
    adafruit-circuitpython-pca9685 \
    adafruit-circuitpython-motor \
    adafruit-circuitpython-servokit \
    pyyaml 2>/dev/null || true

echo "  ✓ Python dependencies installed"

# ─── Step 4: Test I2C ────────────────────────────────────────
echo ""
echo "[Step 4] Testing I2C..."
echo "  Scanning I2C bus 1:"
sudo i2cdetect -y 1 2>/dev/null || echo "  (I2C not available yet - reboot required)"

# ─── Step 5: Setup swap ──────────────────────────────────────
echo ""
echo "[Step 5] Checking swap..."
SWAP_TOTAL=$(free -m | awk '/^Swap:/{print $2}')
echo "  Current swap: ${SWAP_TOTAL}MB"
if [ "$SWAP_TOTAL" -lt 512 ]; then
    echo "  Adding swap..."
    sudo fallocate -l 2G /swapfile 2>/dev/null || sudo dd if=/dev/zero of=/swapfile bs=1M count=2048
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
    sudo swapon /swapfile
    if ! grep -q "/swapfile" /etc/fstab; then
        echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
    fi
    echo "  ✓ Swap added (2GB)"
fi

# ─── Done ─────────────────────────────────────────────────────
echo ""
echo "============================================================="
echo "  ✓ SETUP COMPLETE!"
echo "============================================================="
echo ""
echo "  IMPORTANT: Reboot now!"
echo "    sudo reboot"
echo ""
echo "  After reboot, test I2C:"
echo "    sudo i2cdetect -y 1"
echo ""
echo "  Then test servos:"
echo "    cd ~/x-bot"
echo "    python3 src/assem6_hardware/assem6_hardware/servo_test.py"
echo ""
echo "  Then launch the barista robot:"
echo "    python3 src/assem6_hardware/assem6_hardware/barista_standalone.py"
echo ""
