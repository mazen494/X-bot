#!/usr/bin/env python3
"""
Barista Robot GUI and Controller
Order drinks and the robot moves through stations:
Cup Dispenser -> Ice Dispenser -> Drink Station -> Service Point
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
import time
import math
import sys

# Check for tkinter availability
try:
    import tkinter as tk
    from tkinter import ttk, messagebox
    HAS_TK = True
except ImportError:
    HAS_TK = False
    print("Warning: tkinter not available")


class BaristaRobot(Node):
    def __init__(self):
        super().__init__('barista_robot')
        
        # Joint names
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
        # Current joint positions
        self.current_positions = [0.0, 0.0, 0.0, 0.0]
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Timer to publish joint states
        self.timer = self.create_timer(0.05, self.publish_joint_states)
        
        # Define station positions (joint angles in radians)
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
        
        # Status
        self.is_busy = False
        self.current_task = ""
        self.status_callback = None
        
        self.get_logger().info('Barista Robot initialized!')

    def publish_joint_states(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_positions
        msg.velocity = [0.0] * 6
        msg.effort = [0.0] * 4
        self.joint_pub.publish(msg)

    def move_to_position(self, target_positions, duration=2.0):
        """Smoothly move joints to target positions"""
        start_positions = self.current_positions.copy()
        steps = int(duration * 50)  # 50 Hz update rate
        
        for step in range(steps + 1):
            t = step / steps
            # Smooth interpolation (ease in-out)
            t = t * t * (3 - 2 * t)
            
            for i in range(4):
                self.current_positions[i] = start_positions[i] + t * (target_positions[i] - start_positions[i])
            
            time.sleep(0.02)

    def move_to_station(self, station_name):
        """Move robot to a named station"""
        if station_name in self.stations:
            self.get_logger().info(f'Moving to {station_name}...')
            if self.status_callback:
                self.status_callback(f'Moving to {station_name}...')
            self.move_to_position(self.stations[station_name])
            time.sleep(0.5)  # Pause at station
            return True
        return False

    def execute_single_drink(self, drink_number):
        """Execute a single drink order"""
        drink_station = f'drink{drink_number}'
        
        sequence = [
            ('cup_dispenser', 'Getting cup...'),
            ('ice_dispenser', 'Adding ice...'),
            (drink_station, f'Pouring Drink {drink_number}...'),
            ('service_point', 'Serving...'),
        ]
        
        for station, message in sequence:
            self.get_logger().info(message)
            if self.status_callback:
                self.status_callback(message)
            self.move_to_station(station)
            time.sleep(0.3)

    def execute_order(self, order_dict):
        """Execute the full order with multiple drinks
        order_dict = {1: qty, 2: qty, 3: qty, 4: qty}
        """
        if self.is_busy:
            return False
        
        self.is_busy = True
        
        # Go to home first
        if self.status_callback:
            self.status_callback('Starting order...')
        self.move_to_station('home')
        
        # Process each drink type
        for drink_num in range(1, 5):
            qty = order_dict.get(drink_num, 0)
            for i in range(qty):
                if self.status_callback:
                    self.status_callback(f'Making Drink {drink_num} ({i+1}/{qty})...')
                self.execute_single_drink(drink_num)
        
        # Return home
        if self.status_callback:
            self.status_callback('Order complete!')
        self.move_to_station('home')
        
        self.is_busy = False
        return True


class BaristaGUI:
    def __init__(self, robot_node):
        self.robot = robot_node
        self.root = None
        self.status_label = None
        self.station_label = None
        self.joints_label = None
        
        # Drink quantities
        self.quantities = {1: 0, 2: 0, 3: 0, 4: 0}
        self.qty_labels = {}
        
    def setup(self):
        """Setup the GUI - must be called from main thread"""
        # Create main window
        self.root = tk.Tk()
        self.root.title("Barista Robot")
        self.root.geometry("450x650")
        self.root.configure(bg='#2C3E50')
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # Set status callback
        self.robot.status_callback = self.update_status_safe
        
        self.setup_widgets()
        
    def setup_widgets(self):
        """Setup the GUI elements"""
        # Title
        title_frame = tk.Frame(self.root, bg='#2C3E50')
        title_frame.pack(pady=10)
        
        title_label = tk.Label(
            title_frame,
            text="BARISTA ROBOT",
            font=('Arial', 24, 'bold'),
            fg='#ECF0F1',
            bg='#2C3E50'
        )
        title_label.pack()
        
        subtitle = tk.Label(
            title_frame,
            text="Select drink quantities",
            font=('Arial', 12),
            fg='#BDC3C7',
            bg='#2C3E50'
        )
        subtitle.pack(pady=5)

        # Drinks frame
        drinks_frame = tk.Frame(self.root, bg='#2C3E50')
        drinks_frame.pack(pady=10, padx=20, fill='x')
        
        # Drink buttons with + and - buttons
        drinks = [
            ("Drink 1", "#E74C3C", 1),
            ("Drink 2", "#F39C12", 2),
            ("Drink 3", "#9B59B6", 3),
            ("Drink 4", "#2ECC71", 4),
        ]
        
        for drink_name, color, drink_num in drinks:
            row_frame = tk.Frame(drinks_frame, bg='#2C3E50')
            row_frame.pack(pady=8, fill='x')
            
            # Drink label
            drink_label = tk.Label(
                row_frame,
                text=drink_name,
                font=('Arial', 14, 'bold'),
                fg=color,
                bg='#2C3E50',
                width=10,
                anchor='w'
            )
            drink_label.pack(side='left', padx=10)
            
            # Minus button
            minus_btn = tk.Button(
                row_frame,
                text="-",
                font=('Arial', 16, 'bold'),
                fg='white',
                bg='#E74C3C',
                width=3,
                height=1,
                command=lambda d=drink_num: self.decrease_qty(d)
            )
            minus_btn.pack(side='left', padx=5)
            
            # Quantity label
            qty_label = tk.Label(
                row_frame,
                text="0",
                font=('Arial', 18, 'bold'),
                fg='white',
                bg='#34495E',
                width=4
            )
            qty_label.pack(side='left', padx=5)
            self.qty_labels[drink_num] = qty_label
            
            # Plus button
            plus_btn = tk.Button(
                row_frame,
                text="+",
                font=('Arial', 16, 'bold'),
                fg='white',
                bg='#27AE60',
                width=3,
                height=1,
                command=lambda d=drink_num: self.increase_qty(d)
            )
            plus_btn.pack(side='left', padx=5)

        # Total display
        self.total_label = tk.Label(
            self.root,
            text="Total: 0 cups",
            font=('Arial', 14, 'bold'),
            fg='#F39C12',
            bg='#2C3E50'
        )
        self.total_label.pack(pady=10)

        # Start Order button
        start_btn = tk.Button(
            self.root,
            text="START ORDER",
            font=('Arial', 16, 'bold'),
            fg='white',
            bg='#27AE60',
            activebackground='#2ECC71',
            width=20,
            height=2,
            command=self.start_order
        )
        start_btn.pack(pady=10)
        
        # Clear button
        clear_btn = tk.Button(
            self.root,
            text="Clear Order",
            font=('Arial', 10),
            fg='white',
            bg='#95A5A6',
            command=self.clear_order,
            width=12
        )
        clear_btn.pack(pady=5)

        # Status frame
        status_frame = tk.Frame(self.root, bg='#34495E', relief='ridge', bd=2)
        status_frame.pack(pady=10, padx=20, fill='x')
        
        status_title = tk.Label(
            status_frame,
            text="ROBOT STATUS",
            font=('Arial', 10, 'bold'),
            fg='#BDC3C7',
            bg='#34495E',
        )
        status_title.pack(pady=(10, 5))
        
        self.status_label = tk.Label(
            status_frame,
            text="Ready",
            font=('Arial', 16, 'bold'),
            fg='#2ECC71',
            bg='#34495E',
        )
        self.status_label.pack(pady=5)
        
        # Current station label
        self.station_label = tk.Label(
            status_frame,
            text="Station: Home",
            font=('Arial', 12),
            fg='#3498DB',
            bg='#34495E',
        )
        self.station_label.pack(pady=5)
        
        # Joint positions label
        self.joints_label = tk.Label(
            status_frame,
            text="Joints: [0.00, 0.00, 0.00, 0.00]",
            font=('Courier', 10),
            fg='#95A5A6',
            bg='#34495E',
        )
        self.joints_label.pack(pady=(5, 10))

        # Home button
        home_btn = tk.Button(
            self.root,
            text="Home",
            font=('Arial', 12),
            fg='white',
            bg='#3498DB',
            activebackground='#2980B9',
            command=self.go_home,
            width=10,
            height=1
        )
        home_btn.pack(pady=10)
        
        # Start updating joint positions display
        self.update_joints_display()
    
    def increase_qty(self, drink_num):
        """Increase quantity for a drink"""
        if self.robot.is_busy:
            return
        self.quantities[drink_num] += 1
        self.qty_labels[drink_num].config(text=str(self.quantities[drink_num]))
        self.update_total()
    
    def decrease_qty(self, drink_num):
        """Decrease quantity for a drink"""
        if self.robot.is_busy:
            return
        if self.quantities[drink_num] > 0:
            self.quantities[drink_num] -= 1
            self.qty_labels[drink_num].config(text=str(self.quantities[drink_num]))
            self.update_total()
    
    def update_total(self):
        """Update total cups display"""
        total = sum(self.quantities.values())
        self.total_label.config(text=f"Total: {total} cups")
    
    def clear_order(self):
        """Clear all quantities"""
        if self.robot.is_busy:
            return
        for drink_num in range(1, 5):
            self.quantities[drink_num] = 0
            self.qty_labels[drink_num].config(text="0")
        self.update_total()
    
    def start_order(self):
        """Start the order"""
        if self.robot.is_busy:
            messagebox.showwarning("Busy", "Please wait for current order!")
            return
        
        total = sum(self.quantities.values())
        if total == 0:
            messagebox.showwarning("Empty Order", "Please select at least one drink!")
            return
        
        self.status_label.config(text="Starting order...", fg='#F39C12')
        
        # Run robot movement in separate thread
        thread = threading.Thread(
            target=self.execute_order_thread,
            daemon=True
        )
        thread.start()
    
    def execute_order_thread(self):
        """Execute order in background thread"""
        order_copy = self.quantities.copy()
        self.robot.execute_order(order_copy)
        if self.root:
            self.root.after(0, self.order_complete)
    
    def update_joints_display(self):
        """Update joint positions display periodically"""
        if self.root and self.joints_label:
            pos = self.robot.current_positions
            joints_text = f"Joints: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}, {pos[3]:.2f}]"
            self.joints_label.config(text=joints_text)
            self.root.after(100, self.update_joints_display)

    def order_complete(self):
        """Called when order is complete"""
        if self.status_label:
            self.status_label.config(text="Order Complete!", fg='#2ECC71')
        # Clear the order
        self.clear_order()

    def update_status_safe(self, message):
        """Thread-safe status update"""
        if self.root and self.status_label:
            self.root.after(0, lambda: self.update_status(message))
    
    def update_status(self, message):
        """Update status label"""
        if self.status_label:
            self.status_label.config(text=message)
        # Update station label based on message
        if self.station_label:
            if 'cup' in message.lower():
                self.station_label.config(text="Station: Cup Dispenser")
            elif 'ice' in message.lower():
                self.station_label.config(text="Station: Ice Dispenser")
            elif 'pouring' in message.lower():
                self.station_label.config(text="Station: Drink Dispenser")
            elif 'serv' in message.lower():
                self.station_label.config(text="Station: Service Point")
            elif 'complete' in message.lower():
                self.station_label.config(text="Station: Home")

    def go_home(self):
        """Move robot to home position"""
        if self.robot.is_busy:
            messagebox.showwarning("Busy", "Please wait!")
            return
        
        self.status_label.config(text="Going home...", fg='#F39C12')
        thread = threading.Thread(
            target=self._go_home_thread,
            daemon=True
        )
        thread.start()
    
    def _go_home_thread(self):
        """Go home in background thread"""
        self.robot.move_to_station('home')
        if self.root:
            self.root.after(0, lambda: self.status_label.config(text="Ready", fg='#2ECC71'))

    def on_closing(self):
        """Handle window close"""
        self.root.quit()
        self.root.destroy()

    def run(self):
        """Start the GUI main loop"""
        if self.root:
            self.root.mainloop()


def ros_spin_thread(node):
    """Spin ROS node in separate thread"""
    try:
        rclpy.spin(node)
    except:
        pass


def main():
    rclpy.init()
    
    # Create robot node
    robot = BaristaRobot()
    
    # Start ROS spinning in background thread
    ros_thread = threading.Thread(target=ros_spin_thread, args=(robot,), daemon=True)
    ros_thread.start()
    
    if HAS_TK:
        # Create and run GUI in main thread
        gui = BaristaGUI(robot)
        gui.setup()
        
        try:
            gui.run()
        except KeyboardInterrupt:
            pass
    else:
        # No GUI - just spin
        print("Running without GUI. Press Ctrl+C to exit.")
        try:
            while rclpy.ok():
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
    
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
