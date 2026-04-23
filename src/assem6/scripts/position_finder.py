#!/usr/bin/env python3
"""
Joint Position Finder Tool
Use sliders to find the correct joint positions for each station,
then copy the values to use in barista_gui.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading
import time

try:
    import tkinter as tk
    from tkinter import ttk
    HAS_TK = True
except ImportError:
    HAS_TK = False


class JointPositionFinder(Node):
    def __init__(self):
        super().__init__('joint_position_finder')
        
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.current_positions = [0.0, 0.0, 0.0, 0.0]
        
        # Joint limits from URDF
        self.joint_limits = {
            'joint1': (-3.14, 3.14),
            'joint2': (-1.57, 1.57),
            'joint3': (-1.57, 1.57),
            'joint4': (-3.14, 3.14),
        }
        
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.05, self.publish_joint_states)
        
        # Saved positions
        self.saved_positions = {}
        
        self.get_logger().info('Joint Position Finder initialized!')

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_positions
        msg.velocity = [0.0] * 4
        msg.effort = [0.0] * 4
        self.joint_pub.publish(msg)

    def set_joint(self, joint_index, value):
        self.current_positions[joint_index] = value


class PositionFinderGUI:
    def __init__(self, robot_node):
        self.robot = robot_node
        self.root = None
        self.sliders = []
        self.value_labels = []
        
    def setup(self):
        self.root = tk.Tk()
        self.root.title("Joint Position Finder")
        self.root.geometry("600x700")
        self.root.configure(bg='#1a1a2e')
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        self.setup_widgets()
        
    def setup_widgets(self):
        # Title
        title = tk.Label(
            self.root,
            text="JOINT POSITION FINDER",
            font=('Arial', 20, 'bold'),
            fg='#eee',
            bg='#1a1a2e'
        )
        title.pack(pady=15)
        
        # Instructions
        instr = tk.Label(
            self.root,
            text="Move sliders to find positions, then click Save",
            font=('Arial', 10),
            fg='#aaa',
            bg='#1a1a2e'
        )
        instr.pack(pady=5)
        
        # Sliders frame
        sliders_frame = tk.Frame(self.root, bg='#1a1a2e')
        sliders_frame.pack(pady=20, padx=20, fill='x')
        
        joint_colors = ['#e74c3c', '#f39c12', '#2ecc71', '#3498db']
        
        for i, joint_name in enumerate(self.robot.joint_names):
            limits = self.robot.joint_limits[joint_name]
            
            frame = tk.Frame(sliders_frame, bg='#16213e', relief='ridge', bd=1)
            frame.pack(pady=8, fill='x', padx=10)
            
            # Joint label
            label = tk.Label(
                frame,
                text=f"{joint_name.upper()}",
                font=('Arial', 12, 'bold'),
                fg=joint_colors[i],
                bg='#16213e',
                width=8
            )
            label.pack(side='left', padx=10, pady=10)
            
            # Slider
            slider = tk.Scale(
                frame,
                from_=limits[0],
                to=limits[1],
                resolution=0.01,
                orient='horizontal',
                length=350,
                bg='#16213e',
                fg='white',
                highlightthickness=0,
                troughcolor='#0f3460',
                command=lambda val, idx=i: self.on_slider_change(idx, float(val))
            )
            slider.set(0)
            slider.pack(side='left', padx=10, pady=10)
            self.sliders.append(slider)
            
            # Value label
            val_label = tk.Label(
                frame,
                text="0.00",
                font=('Arial', 12, 'bold'),
                fg='white',
                bg='#16213e',
                width=6
            )
            val_label.pack(side='left', padx=10)
            self.value_labels.append(val_label)
        
        # Current position display
        self.position_display = tk.Text(
            self.root,
            height=3,
            width=50,
            font=('Courier', 11),
            bg='#0f3460',
            fg='#2ecc71',
            relief='flat'
        )
        self.position_display.pack(pady=15)
        self.update_position_display()
        
        # Station buttons frame
        stations_frame = tk.Frame(self.root, bg='#1a1a2e')
        stations_frame.pack(pady=10)
        
        stations = [
            ('Home', '#95a5a6'),
            ('Cup', '#e74c3c'),
            ('Ice', '#3498db'),
            ('Drink 1', '#e74c3c'),
            ('Drink 2', '#f39c12'),
            ('Drink 3', '#9b59b6'),
            ('Drink 4', '#2ecc71'),
            ('Service', '#1abc9c'),
        ]
        
        # Create 2 rows of buttons
        row1 = tk.Frame(stations_frame, bg='#1a1a2e')
        row1.pack(pady=5)
        row2 = tk.Frame(stations_frame, bg='#1a1a2e')
        row2.pack(pady=5)
        
        for i, (name, color) in enumerate(stations):
            parent = row1 if i < 4 else row2
            btn = tk.Button(
                parent,
                text=f"Save\n{name}",
                font=('Arial', 9, 'bold'),
                fg='white',
                bg=color,
                width=8,
                height=2,
                command=lambda n=name: self.save_position(n)
            )
            btn.pack(side='left', padx=5)
        
        # Saved positions display
        tk.Label(
            self.root,
            text="SAVED POSITIONS (copy to barista_gui.py):",
            font=('Arial', 10, 'bold'),
            fg='#f39c12',
            bg='#1a1a2e'
        ).pack(pady=(20, 5))
        
        self.saved_display = tk.Text(
            self.root,
            height=12,
            width=60,
            font=('Courier', 10),
            bg='#0f3460',
            fg='#eee',
            relief='flat'
        )
        self.saved_display.pack(pady=5)
        
        # Reset button
        reset_btn = tk.Button(
            self.root,
            text="Reset All to Zero",
            font=('Arial', 10),
            fg='white',
            bg='#e74c3c',
            command=self.reset_all
        )
        reset_btn.pack(pady=10)

    def on_slider_change(self, joint_index, value):
        self.robot.set_joint(joint_index, value)
        self.value_labels[joint_index].config(text=f"{value:.2f}")
        self.update_position_display()

    def update_position_display(self):
        pos = self.robot.current_positions
        text = f"Current: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}, {pos[3]:.2f}]"
        self.position_display.delete('1.0', 'end')
        self.position_display.insert('1.0', text)

    def save_position(self, station_name):
        pos = self.robot.current_positions.copy()
        self.robot.saved_positions[station_name] = pos
        self.update_saved_display()

    def update_saved_display(self):
        self.saved_display.delete('1.0', 'end')
        
        # Map GUI names to code names
        name_map = {
            'Home': 'home',
            'Cup': 'cup_dispenser',
            'Ice': 'ice_dispenser',
            'Drink 1': 'drink1',
            'Drink 2': 'drink2',
            'Drink 3': 'drink3',
            'Drink 4': 'drink4',
            'Service': 'service_point',
        }
        
        self.saved_display.insert('end', "self.stations = {\n")
        
        for gui_name, code_name in name_map.items():
            if gui_name in self.robot.saved_positions:
                pos = self.robot.saved_positions[gui_name]
                line = f"    '{code_name}': [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}, {pos[3]:.2f}],\n"
                self.saved_display.insert('end', line)
        
        self.saved_display.insert('end', "}\n")

    def reset_all(self):
        for i, slider in enumerate(self.sliders):
            slider.set(0)
            self.robot.set_joint(i, 0)
            self.value_labels[i].config(text="0.00")
        self.update_position_display()

    def on_closing(self):
        self.root.quit()
        self.root.destroy()

    def run(self):
        if self.root:
            self.root.mainloop()


def ros_spin_thread(node):
    try:
        rclpy.spin(node)
    except:
        pass


def main():
    rclpy.init()
    
    robot = JointPositionFinder()
    
    ros_thread = threading.Thread(target=ros_spin_thread, args=(robot,), daemon=True)
    ros_thread.start()
    
    if HAS_TK:
        gui = PositionFinderGUI(robot)
        gui.setup()
        
        try:
            gui.run()
        except KeyboardInterrupt:
            pass
    else:
        print("Tkinter not available")
    
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
