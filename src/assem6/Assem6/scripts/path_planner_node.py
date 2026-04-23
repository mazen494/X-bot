#!/usr/bin/env python3
"""
ROS 2 Path Planning Node for Assem6 Robot Arm
Uses inverse kinematics to plan and execute trajectories.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Float64MultiArray
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import sys
import os

# Add the scripts directory to the path for importing ik_solver
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

from ik_solver import IKSolver


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner')
        
        # Initialize IK solver
        self.ik_solver = IKSolver()
        
        # Current joint states
        self.current_joints = [0.0, 0.0, 0.0, 0.0]
        
        # Publishers
        self.joint_pub = self.create_publisher(
            JointState, 
            '/joint_states', 
            10
        )
        
        self.trajectory_pub = self.create_publisher(
            Float64MultiArray,
            '/planned_trajectory',
            10
        )
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/path_visualization',
            10
        )
        
        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        self.waypoint_sub = self.create_subscription(
            Float64MultiArray,
            '/waypoints',
            self.waypoints_callback,
            10
        )
        
        # Timer for trajectory execution
        self.trajectory_timer = None
        self.current_trajectory = []
        self.trajectory_index = 0
        
        # Joint names from URDF
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
        # Print workspace information
        self.get_logger().info('Path Planner Node initialized')
        home_pos = self.ik_solver.forward_kinematics(self.current_joints)
        self.get_logger().info(f'Home end-effector position: x={home_pos[0]:.3f}, y={home_pos[1]:.3f}, z={home_pos[2]:.3f}')
        
        # Calculate and print workspace bounds
        samples = self.ik_solver.get_workspace_sample()
        self.get_logger().info(f'Workspace X: [{samples[:,0].min():.3f}, {samples[:,0].max():.3f}]')
        self.get_logger().info(f'Workspace Y: [{samples[:,1].min():.3f}, {samples[:,1].max():.3f}]')
        self.get_logger().info(f'Workspace Z: [{samples[:,2].min():.3f}, {samples[:,2].max():.3f}]')
        
        self.get_logger().info('Listening for goal poses on /goal_pose')
        self.get_logger().info('Listening for waypoints on /waypoints')
        
        # Publish initial joint state
        self.publish_joint_state(self.current_joints)
    
    def publish_joint_state(self, joint_angles):
        """Publish joint state message."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = list(joint_angles)
        msg.velocity = [0.0] * 4
        msg.effort = [0.0] * 4
        self.joint_pub.publish(msg)
    
    def goal_callback(self, msg):
        """Handle single goal pose requests."""
        target = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        
        self.get_logger().info(f'Received goal: x={target[0]:.3f}, y={target[1]:.3f}, z={target[2]:.3f}')
        
        # Get current end-effector position
        current_pos = self.ik_solver.forward_kinematics(self.current_joints)
        self.get_logger().info(f'Current position: x={current_pos[0]:.3f}, y={current_pos[1]:.3f}, z={current_pos[2]:.3f}')
        
        # Generate linear path
        waypoints = self.ik_solver.interpolate_path(current_pos, target, num_points=50)
        
        # Plan trajectory using IK
        trajectory = self.ik_solver.plan_cartesian_path(waypoints, self.current_joints)
        
        if trajectory is not None:
            self.get_logger().info(f'Path planned with {len(trajectory)} points')
            self.execute_trajectory(trajectory)
            self.visualize_path(waypoints)
        else:
            self.get_logger().error('Failed to plan path to goal - target may be outside workspace')
    
    def waypoints_callback(self, msg):
        """Handle multiple waypoint requests."""
        # Parse waypoints from flat array [x1,y1,z1, x2,y2,z2, ...]
        data = msg.data
        if len(data) % 3 != 0:
            self.get_logger().error('Invalid waypoints format. Expected [x1,y1,z1, x2,y2,z2, ...]')
            return
        
        waypoints = []
        for i in range(0, len(data), 3):
            waypoints.append(np.array([data[i], data[i+1], data[i+2]]))
        
        self.get_logger().info(f'Received {len(waypoints)} waypoints')
        
        # Interpolate between waypoints
        full_path = []
        current_pos = self.ik_solver.forward_kinematics(self.current_joints)
        
        all_waypoints = [current_pos] + waypoints
        for i in range(len(all_waypoints) - 1):
            segment = self.ik_solver.interpolate_path(
                all_waypoints[i], 
                all_waypoints[i+1], 
                num_points=20
            )
            full_path.extend(segment)
        
        # Plan trajectory
        trajectory = self.ik_solver.plan_cartesian_path(full_path, self.current_joints)
        
        if trajectory is not None:
            self.get_logger().info(f'Multi-waypoint path planned with {len(trajectory)} points')
            self.execute_trajectory(trajectory)
            self.visualize_path(full_path)
        else:
            self.get_logger().error('Failed to plan multi-waypoint path')
    
    def execute_trajectory(self, trajectory):
        """Execute the planned trajectory by publishing joint states."""
        self.current_trajectory = trajectory
        self.trajectory_index = 0
        
        # Cancel existing timer if any
        if self.trajectory_timer is not None:
            self.trajectory_timer.cancel()
        
        # Create timer for trajectory execution (50Hz)
        self.trajectory_timer = self.create_timer(0.02, self.trajectory_step)
    
    def trajectory_step(self):
        """Execute one step of the trajectory."""
        if self.trajectory_index >= len(self.current_trajectory):
            self.trajectory_timer.cancel()
            self.trajectory_timer = None
            final_pos = self.ik_solver.forward_kinematics(self.current_joints)
            self.get_logger().info(f'Trajectory complete. Final position: x={final_pos[0]:.3f}, y={final_pos[1]:.3f}, z={final_pos[2]:.3f}')
            return
        
        # Get current joint angles
        joint_angles = self.current_trajectory[self.trajectory_index]
        self.current_joints = list(joint_angles)
        
        # Publish joint state
        self.publish_joint_state(joint_angles)
        
        self.trajectory_index += 1
    
    def visualize_path(self, waypoints):
        """Publish visualization markers for the planned path."""
        marker_array = MarkerArray()
        
        # Path line
        line_marker = Marker()
        line_marker.header.frame_id = 'base_link'
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = 'path'
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.005  # Line width
        line_marker.color.r = 0.0
        line_marker.color.g = 1.0
        line_marker.color.b = 0.0
        line_marker.color.a = 1.0
        
        for wp in waypoints:
            p = Point()
            p.x = float(wp[0])
            p.y = float(wp[1])
            p.z = float(wp[2])
            line_marker.points.append(p)
        
        marker_array.markers.append(line_marker)
        
        # Goal marker
        goal_marker = Marker()
        goal_marker.header.frame_id = 'base_link'
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.ns = 'goal'
        goal_marker.id = 1
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = float(waypoints[-1][0])
        goal_marker.pose.position.y = float(waypoints[-1][1])
        goal_marker.pose.position.z = float(waypoints[-1][2])
        goal_marker.scale.x = 0.02
        goal_marker.scale.y = 0.02
        goal_marker.scale.z = 0.02
        goal_marker.color.r = 1.0
        goal_marker.color.g = 0.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 1.0
        
        marker_array.markers.append(goal_marker)
        
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
