#!/usr/bin/env python3
"""
Standalone Barista Robot Controller for Raspberry Pi 4
=====================================================
No ROS2 required! This runs the barista GUI on the touchscreen
and drives servos directly via PCA9685.

Usage:
    python3 barista_standalone.py
    python3 barista_standalone.py --config /path/to/servo_config.yaml
    python3 barista_standalone.py --fullscreen
    python3 barista_standalone.py --no-servos   (GUI only, no hardware)

Architecture:
    [Touchscreen GUI] → [Servo Driver] → [PCA9685] → [Servos]
    Everything runs in one process, no ROS2/network needed.
"""

import sys
import os
import math
import time
import threading
import argparse

# Add parent paths for imports
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
sys.path.insert(0, os.path.join(SCRIPT_DIR, '..'))

from servo_driver import PCA9685ServoDriver

# Check for tkinter
try:
    import tkinter as tk
    from tkinter import ttk, messagebox
    HAS_TK = True
except ImportError:
    HAS_TK = False
    print("ERROR: tkinter not available. Install with: sudo apt install python3-tk")


class BaristaController:
    """Controls the robot arm — manages station positions and motion."""

    def __init__(self, driver, status_callback=None):
        self.driver = driver
        self.status_callback = status_callback
        self.is_busy = False
        self.stop_requested = False

        # Current positions (radians)
        self.current_positions = [0.0, 0.0, 0.0, 0.0]

        # Station positions (joint angles in radians)
        # These match the positions from barista_gui.py
        self.stations = {
            'home':           [1.26, -0.44, 1.57, -0.43],
            'cup_dispenser':  [1.58, -0.89, 1.57, -0.87],
            'ice_dispenser':  [-0.65, -0.89, 1.57, -0.87],
            'drink1':         [0.79, -0.89, 1.57, -0.87],
            'drink2':         [0.00, -0.89, 1.57, -0.87],
            'drink3':         [2.45, -0.89, 1.57, -0.87],
            'drink4':         [-3.08, -0.89, 1.57, -0.87],
            'service_point':  [-1.54, -0.89, 1.57, -0.87],
        }

    def update_status(self, message):
        """Send status update to GUI."""
        print(f"  [Robot] {message}")
        if self.status_callback:
            self.status_callback(message)

    def _s_curve_ease(self, t):
        """Quintic ease-in-out S-curve.

        Starts slow, accelerates in the middle, decelerates at the end.
        This profile minimizes peak torque and power consumption:

        Speed:  ╭──────╮
               ╱        ╲
              ╱          ╲
        ─────╯            ╰─────
             0%   50%   100%

        Math: 6t^5 - 15t^4 + 10t^3  (quintic Hermite)
        """
        return t * t * t * (t * (6.0 * t - 15.0) + 10.0)

    def move_to_position(self, target_positions, duration=None):
        """Smoothly move joints to target positions using S-curve profile.

        The duration auto-scales based on the largest joint movement:
          - Small move (<30°): 3 seconds
          - Medium move (30-90°): 5 seconds
          - Large move (>90°): 7 seconds

        The S-curve ensures:
          - Gentle start (low torque at rest)
          - Smooth acceleration (gradual power increase)
          - Fast cruise in the middle
          - Gentle deceleration (no sudden stop)
        """
        start_positions = self.current_positions.copy()

        # Auto-calculate duration based on largest joint travel
        if duration is None:
            max_travel = 0.0
            for i in range(4):
                travel = abs(target_positions[i] - start_positions[i])
                max_travel = max(max_travel, travel)

            max_travel_deg = math.degrees(max_travel)
            if max_travel_deg < 10:
                duration = 2.0   # Very small move
            elif max_travel_deg < 30:
                duration = 3.0   # Small move
            elif max_travel_deg < 90:
                duration = 5.0   # Medium move
            elif max_travel_deg < 180:
                duration = 7.0   # Large move
            else:
                duration = 9.0   # Very large move

        steps = int(duration * 50)  # 50 Hz update rate
        if steps < 1:
            steps = 1

        for step in range(steps + 1):
            if self.stop_requested:
                return

            t = step / steps

            # Apply S-curve easing (slow→fast→slow)
            t_eased = self._s_curve_ease(t)

            positions = []
            for i in range(4):
                pos = start_positions[i] + t_eased * (target_positions[i] - start_positions[i])
                positions.append(pos)

            # Send to hardware servos
            self.driver.set_all_angles(positions)
            self.current_positions = positions.copy()

            time.sleep(0.02)  # 50 Hz

    def move_to_station(self, station_name):
        """Move robot to a named station."""
        if station_name in self.stations:
            self.update_status(f'Moving to {station_name}...')
            self.move_to_position(self.stations[station_name])
            time.sleep(1.0)  # Pause at station
            return True
        return False

    def execute_single_drink(self, drink_number):
        """Execute a single drink order."""
        drink_station = f'drink{drink_number}'

        sequence = [
            ('cup_dispenser', 'Getting cup...'),
            ('ice_dispenser', 'Adding ice...'),
            (drink_station, f'Pouring Drink {drink_number}...'),
            ('service_point', 'Serving...'),
        ]

        for station, message in sequence:
            if self.stop_requested:
                return
            self.update_status(message)
            self.move_to_station(station)
            time.sleep(1.0)  # Pause between stations

    def execute_order(self, order_dict):
        """Execute the full order with multiple drinks.

        order_dict = {1: qty, 2: qty, 3: qty, 4: qty}
        """
        if self.is_busy:
            return False

        self.is_busy = True
        self.stop_requested = False

        # Go to home first
        self.update_status('Starting order...')
        self.move_to_station('home')

        # Process each drink type
        for drink_num in range(1, 5):
            if self.stop_requested:
                break
            qty = order_dict.get(drink_num, 0)
            for i in range(qty):
                if self.stop_requested:
                    break
                self.update_status(f'Making Drink {drink_num} ({i+1}/{qty})...')
                self.execute_single_drink(drink_num)

        # Return home
        self.update_status('Order complete!')
        self.move_to_station('home')

        self.is_busy = False
        return True

    def stop(self):
        """Emergency stop."""
        self.stop_requested = True


class BaristaGUI:
    """Touchscreen GUI for the barista robot."""

    def __init__(self, controller, fullscreen=False):
        self.controller = controller
        self.fullscreen = fullscreen
        self.root = None
        self.status_label = None
        self.station_label = None
        self.joints_label = None

        # Drink quantities
        self.quantities = {1: 0, 2: 0, 3: 0, 4: 0}
        self.qty_labels = {}

    def setup(self):
        """Setup the GUI."""
        self.root = tk.Tk()
        self.root.title("BARISTA ROBOT")

        if self.fullscreen:
            self.root.attributes('-fullscreen', True)
            # Press Escape to exit fullscreen
            self.root.bind('<Escape>', lambda e: self.root.attributes('-fullscreen', False))
        else:
            self.root.geometry("480x800")

        self.root.configure(bg='#1a1a2e')
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Set status callback
        self.controller.status_callback = self.update_status_safe

        self.setup_widgets()

    def setup_widgets(self):
        """Setup all GUI elements."""
        # ─── Title ───
        title_frame = tk.Frame(self.root, bg='#1a1a2e')
        title_frame.pack(pady=10)

        title_label = tk.Label(
            title_frame,
            text="☕ BARISTA ROBOT",
            font=('Arial', 26, 'bold'),
            fg='#e94560',
            bg='#1a1a2e'
        )
        title_label.pack()

        subtitle = tk.Label(
            title_frame,
            text="Select your drinks",
            font=('Arial', 12),
            fg='#8899aa',
            bg='#1a1a2e'
        )
        subtitle.pack(pady=3)

        # ─── Drinks ───
        drinks_frame = tk.Frame(self.root, bg='#1a1a2e')
        drinks_frame.pack(pady=10, padx=20, fill='x')

        drinks = [
            ("☕ Drink 1", "#e94560", 1),
            ("🧃 Drink 2", "#f39c12", 2),
            ("🍵 Drink 3", "#9b59b6", 3),
            ("🥤 Drink 4", "#2ecc71", 4),
        ]

        for drink_name, color, drink_num in drinks:
            row_frame = tk.Frame(drinks_frame, bg='#16213e', relief='flat', bd=0)
            row_frame.pack(pady=6, fill='x', ipady=5)

            # Drink label
            drink_label = tk.Label(
                row_frame,
                text=drink_name,
                font=('Arial', 16, 'bold'),
                fg=color,
                bg='#16213e',
                width=10,
                anchor='w'
            )
            drink_label.pack(side='left', padx=15)

            # Minus button
            minus_btn = tk.Button(
                row_frame,
                text="−",
                font=('Arial', 20, 'bold'),
                fg='white',
                bg='#e94560',
                activebackground='#c0392b',
                width=3,
                height=1,
                bd=0,
                command=lambda d=drink_num: self.decrease_qty(d)
            )
            minus_btn.pack(side='left', padx=5)

            # Quantity label
            qty_label = tk.Label(
                row_frame,
                text="0",
                font=('Arial', 22, 'bold'),
                fg='white',
                bg='#0f3460',
                width=4
            )
            qty_label.pack(side='left', padx=8)
            self.qty_labels[drink_num] = qty_label

            # Plus button
            plus_btn = tk.Button(
                row_frame,
                text="+",
                font=('Arial', 20, 'bold'),
                fg='white',
                bg='#27ae60',
                activebackground='#229954',
                width=3,
                height=1,
                bd=0,
                command=lambda d=drink_num: self.increase_qty(d)
            )
            plus_btn.pack(side='left', padx=5)

        # ─── Total ───
        self.total_label = tk.Label(
            self.root,
            text="Total: 0 cups",
            font=('Arial', 16, 'bold'),
            fg='#f39c12',
            bg='#1a1a2e'
        )
        self.total_label.pack(pady=8)

        # ─── Buttons ───
        btn_frame = tk.Frame(self.root, bg='#1a1a2e')
        btn_frame.pack(pady=8)

        # Start Order button
        start_btn = tk.Button(
            btn_frame,
            text="▶  START ORDER",
            font=('Arial', 18, 'bold'),
            fg='white',
            bg='#27ae60',
            activebackground='#229954',
            width=18,
            height=2,
            bd=0,
            command=self.start_order
        )
        start_btn.pack(pady=5)

        # Row of smaller buttons
        small_btn_frame = tk.Frame(btn_frame, bg='#1a1a2e')
        small_btn_frame.pack(pady=5)

        clear_btn = tk.Button(
            small_btn_frame,
            text="Clear",
            font=('Arial', 12),
            fg='white',
            bg='#7f8c8d',
            activebackground='#95a5a6',
            width=8,
            bd=0,
            command=self.clear_order
        )
        clear_btn.pack(side='left', padx=5)

        home_btn = tk.Button(
            small_btn_frame,
            text="🏠 Home",
            font=('Arial', 12),
            fg='white',
            bg='#3498db',
            activebackground='#2980b9',
            width=8,
            bd=0,
            command=self.go_home
        )
        home_btn.pack(side='left', padx=5)

        stop_btn = tk.Button(
            small_btn_frame,
            text="⛔ STOP",
            font=('Arial', 12, 'bold'),
            fg='white',
            bg='#c0392b',
            activebackground='#e74c3c',
            width=8,
            bd=0,
            command=self.emergency_stop
        )
        stop_btn.pack(side='left', padx=5)

        # ─── Status ───
        status_frame = tk.Frame(self.root, bg='#16213e', relief='flat', bd=0)
        status_frame.pack(pady=10, padx=20, fill='x')

        status_title = tk.Label(
            status_frame,
            text="ROBOT STATUS",
            font=('Arial', 10, 'bold'),
            fg='#8899aa',
            bg='#16213e',
        )
        status_title.pack(pady=(10, 3))

        self.status_label = tk.Label(
            status_frame,
            text="✓ Ready",
            font=('Arial', 18, 'bold'),
            fg='#2ecc71',
            bg='#16213e',
        )
        self.status_label.pack(pady=3)

        self.station_label = tk.Label(
            status_frame,
            text="Station: Home",
            font=('Arial', 12),
            fg='#3498db',
            bg='#16213e',
        )
        self.station_label.pack(pady=3)

        self.joints_label = tk.Label(
            status_frame,
            text="Joints: [0.00, 0.00, 0.00, 0.00]",
            font=('Courier', 10),
            fg='#7f8c8d',
            bg='#16213e',
        )
        self.joints_label.pack(pady=(3, 10))

        # Hardware mode indicator
        mode = "SIMULATION" if self.controller.driver.simulation_mode else "HARDWARE"
        mode_color = "#f39c12" if self.controller.driver.simulation_mode else "#2ecc71"
        mode_label = tk.Label(
            self.root,
            text=f"Mode: {mode}",
            font=('Arial', 10),
            fg=mode_color,
            bg='#1a1a2e'
        )
        mode_label.pack(pady=3)

        # Start updating joint display
        self.update_joints_display()

    def increase_qty(self, drink_num):
        if self.controller.is_busy:
            return
        self.quantities[drink_num] += 1
        self.qty_labels[drink_num].config(text=str(self.quantities[drink_num]))
        self.update_total()

    def decrease_qty(self, drink_num):
        if self.controller.is_busy:
            return
        if self.quantities[drink_num] > 0:
            self.quantities[drink_num] -= 1
            self.qty_labels[drink_num].config(text=str(self.quantities[drink_num]))
            self.update_total()

    def update_total(self):
        total = sum(self.quantities.values())
        self.total_label.config(text=f"Total: {total} cups")

    def clear_order(self):
        if self.controller.is_busy:
            return
        for drink_num in range(1, 5):
            self.quantities[drink_num] = 0
            self.qty_labels[drink_num].config(text="0")
        self.update_total()

    def start_order(self):
        if self.controller.is_busy:
            messagebox.showwarning("Busy", "Please wait for the current order!")
            return

        total = sum(self.quantities.values())
        if total == 0:
            messagebox.showwarning("Empty", "Please select at least one drink!")
            return

        self.status_label.config(text="Starting order...", fg='#f39c12')

        thread = threading.Thread(target=self._execute_order_thread, daemon=True)
        thread.start()

    def _execute_order_thread(self):
        order_copy = self.quantities.copy()
        self.controller.execute_order(order_copy)
        if self.root:
            self.root.after(0, self._order_complete)

    def _order_complete(self):
        if self.status_label:
            self.status_label.config(text="✓ Order Complete!", fg='#2ecc71')
        self.clear_order()

    def go_home(self):
        if self.controller.is_busy:
            messagebox.showwarning("Busy", "Please wait!")
            return

        self.status_label.config(text="Going home...", fg='#f39c12')
        thread = threading.Thread(target=self._go_home_thread, daemon=True)
        thread.start()

    def _go_home_thread(self):
        self.controller.move_to_station('home')
        if self.root:
            self.root.after(0, lambda: self.status_label.config(text="✓ Ready", fg='#2ecc71'))

    def emergency_stop(self):
        """Emergency stop — halts all movement immediately."""
        self.controller.stop()
        if self.status_label:
            self.status_label.config(text="⛔ STOPPED", fg='#e74c3c')

    def update_joints_display(self):
        """Update joint positions display periodically."""
        if self.root and self.joints_label:
            pos = self.controller.current_positions
            joints_text = f"Joints: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}, {pos[3]:.2f}]"
            self.joints_label.config(text=joints_text)
            self.root.after(100, self.update_joints_display)

    def update_status_safe(self, message):
        """Thread-safe status update."""
        if self.root and self.status_label:
            self.root.after(0, lambda: self._update_status(message))

    def _update_status(self, message):
        if self.status_label:
            self.status_label.config(text=message)
        if self.station_label:
            msg_lower = message.lower()
            if 'cup' in msg_lower:
                self.station_label.config(text="Station: Cup Dispenser")
            elif 'ice' in msg_lower:
                self.station_label.config(text="Station: Ice Dispenser")
            elif 'pouring' in msg_lower:
                self.station_label.config(text="Station: Drink Dispenser")
            elif 'serv' in msg_lower:
                self.station_label.config(text="Station: Service Point")
            elif 'complete' in msg_lower:
                self.station_label.config(text="Station: Home")
            elif 'home' in msg_lower:
                self.station_label.config(text="Station: Home")

    def on_closing(self):
        self.controller.stop()
        self.controller.driver.shutdown()
        self.root.quit()
        self.root.destroy()

    def run(self):
        if self.root:
            self.root.mainloop()


def main():
    parser = argparse.ArgumentParser(description='Assem6 Barista Robot - Standalone')
    parser.add_argument('--config', type=str, default=None,
                        help='Path to servo_config.yaml')
    parser.add_argument('--fullscreen', action='store_true',
                        help='Run GUI in fullscreen mode (for touchscreen)')
    parser.add_argument('--no-servos', action='store_true',
                        help='Run without servo hardware (GUI only)')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='Speed multiplier (0.5=half speed, 2.0=double speed)')
    args = parser.parse_args()

    # Find config file
    config_path = args.config
    if not config_path:
        for path in [
            os.path.join(SCRIPT_DIR, '..', 'config', 'servo_config.yaml'),
            os.path.join(SCRIPT_DIR, 'config', 'servo_config.yaml'),
            os.path.expanduser('~/x-bot/src/assem6_hardware/config/servo_config.yaml'),
        ]:
            if os.path.exists(path):
                config_path = os.path.abspath(path)
                break

    print("=" * 50)
    print("  ASSEM6 BARISTA ROBOT - Standalone Mode")
    print("=" * 50)

    # Initialize servo driver
    driver = PCA9685ServoDriver(config_path)

    if args.no_servos:
        driver.simulation_mode = True
        driver.initialized = True
        print("  Mode: NO SERVOS (GUI only)")
    else:
        driver.initialize()

    driver.print_status()

    # Create controller
    controller = BaristaController(driver)

    if not HAS_TK:
        print("ERROR: tkinter not installed!")
        print("Install with: sudo apt install python3-tk")
        driver.shutdown()
        sys.exit(1)

    # Create and run GUI
    gui = BaristaGUI(controller, fullscreen=args.fullscreen)
    gui.setup()

    print("\n  ✓ GUI started!")
    print("  Press Ctrl+C or close the window to exit.\n")

    try:
        gui.run()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        driver.shutdown()

    print("Goodbye!")


if __name__ == '__main__':
    main()
