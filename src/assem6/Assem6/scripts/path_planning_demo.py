#!/usr/bin/env python3
"""
Demo script to test path planning with inverse kinematics.
Sends goal positions to the path planner node.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import time


class PathPlanningDemo(Node):
    def __init__(self):
        super().__init__('path_planning_demo')
        
        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.waypoints_pub = self.create_publisher(Float64MultiArray, '/waypoints', 10)
        
        self.get_logger().info('Path Planning Demo initialized')
        self.get_logger().info('Waiting for path planner to be ready...')
        
        # Wait for connections
        time.sleep(2.0)
        
        # Run demo
        self.run_demo()
    
    def send_goal(self, x, y, z):
        """Send a single goal position."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        
        self.goal_pub.publish(msg)
        self.get_logger().info(f'Sent goal: ({x:.3f}, {y:.3f}, {z:.3f})')
    
    def send_waypoints(self, waypoints):
        """Send multiple waypoints as a flat array."""
        msg = Float64MultiArray()
        flat_data = []
        for wp in waypoints:
            flat_data.extend(wp)
        msg.data = flat_data
        
        self.waypoints_pub.publish(msg)
        self.get_logger().info(f'Sent {len(waypoints)} waypoints')
    
    def run_demo(self):
        """Run the path planning demonstration with reachable positions."""
        self.get_logger().info('=' * 50)
        self.get_logger().info('Starting Path Planning Demo')
        self.get_logger().info('Using positions within robot workspace')
        self.get_logger().info('=' * 50)
        
        # Demo 1: Move to a reachable position (within workspace bounds)
        # Workspace: X[-0.36, 0.43], Y[-0.48, 0.31], Z[-0.13, 0.49]
        self.get_logger().info('\n--- Demo 1: Moving to reachable goal ---')
        self.send_goal(0.15, -0.1, 0.2)
        time.sleep(3.0)
        
        # Demo 2: Another reachable position
        self.get_logger().info('\n--- Demo 2: Moving to another position ---')
        self.send_goal(0.2, 0.1, 0.15)
        time.sleep(3.0)
        
        # Demo 3: Multiple waypoints (small square pattern in reachable space)
        self.get_logger().info('\n--- Demo 3: Following waypoints (square pattern) ---')
        waypoints = [
            [0.15, 0.0, 0.2],    # Point 1
            [0.15, -0.15, 0.2],  # Point 2
            [0.25, -0.15, 0.2],  # Point 3
            [0.25, 0.0, 0.2],    # Point 4
            [0.15, 0.0, 0.2],    # Back to Point 1
        ]
        self.send_waypoints(waypoints)
        time.sleep(5.0)
        
        # Demo 4: Vertical motion
        self.get_logger().info('\n--- Demo 4: Vertical motion ---')
        self.send_goal(0.2, -0.05, 0.35)
        time.sleep(3.0)
        
        self.send_goal(0.2, -0.05, 0.1)
        time.sleep(3.0)
        
        # Demo 5: Return to home-ish position
        self.get_logger().info('\n--- Demo 5: Return near home ---')
        self.send_goal(0.05, -0.06, 0.1)
        time.sleep(3.0)
        
        self.get_logger().info('\n--- Demo Complete ---')


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningDemo()
    
    try:
        rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
