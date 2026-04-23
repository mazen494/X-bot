#!/usr/bin/env python3
"""
Scenario Runner for Assem6 Robot Arm
Executes a continuous sequence of poses using MoveIt.
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
import time


class ScenarioRunner(Node):
    def __init__(self):
        super().__init__('scenario_runner')
        
        # Publisher for joint trajectory
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # Also try the arm controller topic
        self.arm_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )
        
        # Fallback: publish to joint_states for visualization
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Define your poses (joint values from SRDF)
        self.poses = {
            "home": [0.0, 0.0, 0.0, 0.0],
            "p1": [-1.5794, 0.7113, -1.57, 0.781],
            "p2": [-1.57, 0.8414, -1.4312, 0.781],
            "p3": [-0.8157, 0.6852, -1.57, 0.538],
            "p4": [0.0521, 0.8848, -1.57, 0.8505],
            "p5": [0.0, 0.9541, -1.5006, 0.8505],
            "p6": [1.51, 0.9541, -1.5006, 0.8505],
            "p7": [1.57, 0.6679, -0.8587, 0.0],
        }
        
        self.joint_names = ["joint1", "joint2", "joint3", "joint4"]
        self.current_position = self.poses["home"].copy()
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("Scenario Runner initialized!")
        self.get_logger().info(f"Available poses: {', '.join(self.poses.keys())}")
        self.get_logger().info("=" * 50)
    
    def interpolate(self, start, end, steps=50):
        """Generate interpolated positions between start and end."""
        trajectory = []
        for i in range(steps + 1):
            t = i / steps
            point = [s + t * (e - s) for s, e in zip(start, end)]
            trajectory.append(point)
        return trajectory
    
    def publish_joint_state(self, positions):
        """Publish joint state for visualization."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = positions
        msg.velocity = [0.0] * 4
        msg.effort = [0.0] * 4
        self.joint_state_pub.publish(msg)
    
    def move_to_pose(self, pose_name, duration=2.0):
        """Move smoothly to a named pose."""
        if pose_name not in self.poses:
            self.get_logger().error(f"Unknown pose: {pose_name}")
            return False
        
        target = self.poses[pose_name]
        self.get_logger().info(f"Moving to: {pose_name}")
        
        # Generate smooth trajectory
        steps = int(duration * 50)  # 50 Hz update rate
        trajectory = self.interpolate(self.current_position, target, steps)
        
        # Execute trajectory by publishing joint states
        for i, positions in enumerate(trajectory):
            self.publish_joint_state(positions)
            time.sleep(duration / steps)
        
        self.current_position = target.copy()
        self.get_logger().info(f"✓ Reached: {pose_name}")
        return True
    
    def run_scenario(self, pose_sequence, move_duration=2.0, pause_between=0.5):
        """Run a continuous sequence of poses."""
        self.get_logger().info("\n" + "=" * 50)
        self.get_logger().info("Starting Continuous Scenario")
        self.get_logger().info(f"Sequence: {' → '.join(pose_sequence)}")
        self.get_logger().info(f"Move duration: {move_duration}s, Pause: {pause_between}s")
        self.get_logger().info("=" * 50 + "\n")
        
        # First, set initial position
        self.current_position = self.poses.get(pose_sequence[0], self.poses["home"]).copy()
        self.publish_joint_state(self.current_position)
        time.sleep(0.5)
        
        for i, pose in enumerate(pose_sequence):
            self.get_logger().info(f"\n--- Step {i+1}/{len(pose_sequence)}: {pose} ---")
            self.move_to_pose(pose, move_duration)
            
            if pause_between > 0 and i < len(pose_sequence) - 1:
                time.sleep(pause_between)
        
        self.get_logger().info("\n" + "=" * 50)
        self.get_logger().info("✓ Scenario Complete!")
        self.get_logger().info("=" * 50)
        return True
    
    def run_loop(self, pose_sequence, loops=1, move_duration=2.0, pause_between=0.5):
        """Run the scenario multiple times."""
        for loop in range(loops):
            self.get_logger().info(f"\n{'#' * 50}")
            self.get_logger().info(f"Loop {loop + 1}/{loops}")
            self.get_logger().info('#' * 50)
            self.run_scenario(pose_sequence, move_duration, pause_between)
            
            if loop < loops - 1:
                time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    
    runner = ScenarioRunner()
    
    # ============================================
    # DEFINE YOUR SCENARIO HERE
    # ============================================
    
    # Continuous sequence: robot moves through all poses in order
    scenario = [
        "home",
        "p1",
        "p2",
        "p3",
        "p4",
        "p5",
        "p6",
        "p7",
        "home",  # Return to home at the end
    ]
    
    # Settings
    MOVE_DURATION = 20.0      # Slower motion (5 seconds per move)
    PAUSE_BETWEEN = 5.0      # Longer pause at each pose
    NUMBER_OF_LOOPS = 3      # Repeat 3 times
    
    try:
        runner.run_loop(
            scenario, 
            loops=NUMBER_OF_LOOPS,
            move_duration=MOVE_DURATION, 
            pause_between=PAUSE_BETWEEN
        )
    except KeyboardInterrupt:
        runner.get_logger().info("\nScenario interrupted by user")
    
    runner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
