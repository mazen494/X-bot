#!/usr/bin/env python3
"""
Interactive Joint Controller for assem6 robot
Controls joints directly via Gazebo service
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetModelConfiguration
from std_msgs.msg import Float64
import sys


class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        
        # Joint names for assem6 robot
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
        # Current target positions (in radians)
        self.positions = [0.0, 0.0, 0.0, 0.0]
        
        # Create service client for setting joint positions
        self.set_config_client = self.create_client(
            SetModelConfiguration,
            '/gazebo/set_model_configuration'
        )
        
        self.get_logger().info('Joint Controller initialized!')
        self.get_logger().info(f'Controlling joints: {self.joint_names}')
        
        # Wait for service
        print("Waiting for Gazebo service...")
        if self.set_config_client.wait_for_service(timeout_sec=10.0):
            print("Connected to Gazebo!")
        else:
            print("Warning: Gazebo service not available. Make sure Gazebo is running.")
        
        self.print_help()

    def print_help(self):
        print("\n" + "="*50)
        print("JOINT CONTROLLER - GAZEBO")
        print("="*50)
        print("Commands:")
        print("  1 <angle>  - Set joint1 angle (radians)")
        print("  2 <angle>  - Set joint2 angle (radians)")
        print("  3 <angle>  - Set joint3 angle (radians)")
        print("  4 <angle>  - Set joint4 angle (radians)")
        print("  all <j1> <j2> <j3> <j4> - Set all joints")
        print("  home       - Move all joints to 0")
        print("  status     - Show current target positions")
        print("  help       - Show this help")
        print("  quit       - Exit")
        print("="*50)
        print(f"Current positions: {self.positions}")
        print("="*50 + "\n")

    def send_joint_positions(self):
        """Send joint positions to Gazebo"""
        if not self.set_config_client.service_is_ready():
            print("Error: Gazebo service not available!")
            return False
        
        request = SetModelConfiguration.Request()
        request.model_name = 'assem6'
        request.urdf_param_name = 'robot_description'
        request.joint_names = self.joint_names
        request.joint_positions = self.positions
        
        future = self.set_config_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            if future.result().success:
                print(f"✓ Joints moved to: {[f'{p:.2f}' for p in self.positions]}")
                return True
            else:
                print(f"✗ Failed: {future.result().status_message}")
                return False
        else:
            print("✗ Service call timed out")
            return False

    def process_command(self, cmd):
        """Process user command"""
        parts = cmd.strip().split()
        if not parts:
            return True
        
        command = parts[0].lower()
        
        try:
            if command == 'quit' or command == 'q':
                return False
            
            elif command == 'help' or command == 'h':
                self.print_help()
            
            elif command == 'status' or command == 's':
                print(f"\nCurrent target positions:")
                for i, (name, pos) in enumerate(zip(self.joint_names, self.positions)):
                    print(f"  {name}: {pos:.3f} rad ({pos * 57.2958:.1f}°)")
                print()
            
            elif command == 'home':
                self.positions = [0.0, 0.0, 0.0, 0.0]
                self.send_joint_positions()
            
            elif command == 'all':
                if len(parts) >= 5:
                    self.positions = [float(parts[i]) for i in range(1, 5)]
                    self.send_joint_positions()
                else:
                    print("Usage: all <j1> <j2> <j3> <j4>")
                    print("Example: all 0.5 0.3 -0.2 1.0")
            
            elif command in ['1', '2', '3', '4']:
                joint_idx = int(command) - 1
                if len(parts) >= 2:
                    angle = float(parts[1])
                    self.positions[joint_idx] = angle
                    self.send_joint_positions()
                else:
                    print(f"Usage: {command} <angle_in_radians>")
                    print(f"Example: {command} 0.5")
            
            else:
                print(f"Unknown command: '{command}'. Type 'help' for available commands.")
        
        except ValueError as e:
            print(f"Error: Invalid number - {e}")
        except Exception as e:
            print(f"Error: {e}")
        
        return True


def main():
    rclpy.init()
    node = JointController()
    
    print("\nReady to control robot joints!")
    print("Type 'help' for available commands.\n")
    
    try:
        while True:
            try:
                cmd = input(">>> ")
                if not node.process_command(cmd):
                    break
            except EOFError:
                break
    except KeyboardInterrupt:
        print("\nShutting down...")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
