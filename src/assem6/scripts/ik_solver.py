#!/usr/bin/env python3
"""
Inverse Kinematics Solver for Assem6 4-DOF Robot Arm
This module provides IK solving capabilities using numerical methods.
"""

import numpy as np
from scipy.optimize import minimize


def rotation_x(angle):
    """Rotation matrix around X axis."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [1, 0, 0, 0],
        [0, c, -s, 0],
        [0, s, c, 0],
        [0, 0, 0, 1]
    ])


def rotation_z(angle):
    """Rotation matrix around Z axis."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [c, -s, 0, 0],
        [s, c, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])


def translation(x, y, z):
    """Translation matrix."""
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])


class IKSolver:
    def __init__(self):
        # Joint limits from URDF (axis is -Z, so we need to handle sign)
        self.joint_limits = [
            (-6.28, 6.28),    # joint1: continuous
            (-1.57, 1.57),    # joint2: revolute (expanded for reachability)
            (-1.57, 1.57),    # joint3: revolute (expanded for reachability)
            (-3.14, 3.14),    # joint4: continuous
        ]
        
        # Joint origins from URDF
        self.joint1_origin = np.array([0.033986, -0.081844, 0.011835])
        self.joint2_origin = np.array([0, -0.0625, 0.09129])
        self.joint3_origin = np.array([0, 0.378542, 0.0288])
        self.joint4_origin = np.array([0, 0.39129, 0.1158])
        
        # End effector offset (approximate)
        self.ee_offset = np.array([0, 0.05, 0])
    
    def forward_kinematics(self, joint_angles):
        """
        Calculate end-effector position from joint angles using transformation matrices.
        Follows the exact URDF structure.
        
        Args:
            joint_angles: List of 4 joint angles [q1, q2, q3, q4]
        
        Returns:
            End-effector position [x, y, z]
        """
        q1, q2, q3, q4 = joint_angles
        
        # Base to Joint1 origin
        T_base = translation(*self.joint1_origin)
        
        # Joint1 rotation (around -Z axis)
        T_j1 = rotation_z(-q1)
        
        # Joint1 to Joint2 origin + rotation (includes 90° X rotation from URDF)
        T_12 = translation(*self.joint2_origin) @ rotation_x(np.pi/2)
        
        # Joint2 rotation (around -Z axis)
        T_j2 = rotation_z(-q2)
        
        # Joint2 to Joint3 origin + rotation (includes 180° rotation from URDF)
        T_23 = translation(*self.joint3_origin) @ rotation_x(-np.pi)
        
        # Joint3 rotation (around -Z axis)
        T_j3 = rotation_z(-q3)
        
        # Joint3 to Joint4 origin
        T_34 = translation(*self.joint4_origin)
        
        # Joint4 rotation (around -Z axis)
        T_j4 = rotation_z(-q4)
        
        # End effector offset
        T_ee = translation(*self.ee_offset)
        
        # Complete transformation
        T_total = T_base @ T_j1 @ T_12 @ T_j2 @ T_23 @ T_j3 @ T_34 @ T_j4 @ T_ee
        
        # Extract position
        position = T_total[:3, 3]
        
        return position
    
    def inverse_kinematics(self, target_position, initial_guess=None):
        """
        Calculate joint angles to reach target position using numerical optimization.
        
        Args:
            target_position: Desired [x, y, z] position
            initial_guess: Initial joint angles (optional)
        
        Returns:
            Joint angles [q1, q2, q3, q4] or None if no solution found
        """
        target_position = np.array(target_position)
        
        if initial_guess is None:
            initial_guess = [0.0, 0.0, 0.0, 0.0]
        
        def objective(q):
            current_pos = self.forward_kinematics(q)
            error = np.linalg.norm(current_pos - target_position)
            return error
        
        # Bounds from joint limits
        bounds = self.joint_limits
        
        # Try multiple initial guesses for better convergence
        best_solution = None
        best_error = float('inf')
        
        initial_guesses = [
            initial_guess,
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.5, 0.5, 0.0],
            [0.0, -0.5, -0.5, 0.0],
            [1.57, 0.0, 0.0, 0.0],
            [-1.57, 0.0, 0.0, 0.0],
            [3.14, 0.5, 0.5, 0.0],
            [0.0, 1.0, -1.0, 0.0],
            [0.0, -1.0, 1.0, 0.0],
        ]
        
        for guess in initial_guesses:
            try:
                result = minimize(
                    objective,
                    guess,
                    method='SLSQP',
                    bounds=bounds,
                    options={'ftol': 1e-8, 'maxiter': 500}
                )
                
                if result.fun < best_error:
                    best_error = result.fun
                    best_solution = result.x
            except Exception as e:
                continue
        
        # Check if solution is acceptable (within 5mm tolerance)
        if best_error < 0.005:
            return best_solution
        elif best_error < 0.02:
            print(f"Warning: IK solution has error of {best_error*1000:.2f}mm")
            return best_solution
        else:
            print(f"Error: Could not find IK solution. Best error: {best_error*1000:.2f}mm")
            print(f"Target: {target_position}, Closest: {self.forward_kinematics(best_solution)}")
            return None
    
    def get_workspace_sample(self):
        """Get sample reachable positions in the workspace."""
        samples = []
        for q1 in np.linspace(-3.14, 3.14, 5):
            for q2 in np.linspace(-1.0, 1.0, 5):
                for q3 in np.linspace(-1.0, 1.0, 5):
                    pos = self.forward_kinematics([q1, q2, q3, 0])
                    samples.append(pos)
        return np.array(samples)
    
    def print_workspace_bounds(self):
        """Print the approximate workspace bounds."""
        samples = self.get_workspace_sample()
        print(f"Workspace X: [{samples[:,0].min():.3f}, {samples[:,0].max():.3f}]")
        print(f"Workspace Y: [{samples[:,1].min():.3f}, {samples[:,1].max():.3f}]")
        print(f"Workspace Z: [{samples[:,2].min():.3f}, {samples[:,2].max():.3f}]")
    
    def interpolate_path(self, start_pos, end_pos, num_points=50):
        """
        Generate a linear path between two positions.
        
        Args:
            start_pos: Starting [x, y, z] position
            end_pos: Ending [x, y, z] position
            num_points: Number of waypoints
        
        Returns:
            List of [x, y, z] waypoints
        """
        start_pos = np.array(start_pos)
        end_pos = np.array(end_pos)
        path = []
        for i in range(num_points):
            t = i / (num_points - 1)
            point = start_pos + t * (end_pos - start_pos)
            path.append(point)
        return path
    
    def plan_cartesian_path(self, waypoints, initial_joints=None):
        """
        Plan joint trajectory for a series of Cartesian waypoints.
        
        Args:
            waypoints: List of [x, y, z] positions
            initial_joints: Starting joint configuration
        
        Returns:
            List of joint angle arrays for each waypoint
        """
        joint_trajectory = []
        current_joints = initial_joints if initial_joints else [0.0, 0.0, 0.0, 0.0]
        
        for i, waypoint in enumerate(waypoints):
            joint_angles = self.inverse_kinematics(waypoint, current_joints)
            if joint_angles is not None:
                joint_trajectory.append(joint_angles)
                current_joints = list(joint_angles)
            else:
                print(f"Failed to compute IK for waypoint {i}: {waypoint}")
                return None
        
        return joint_trajectory


# Test the solver when run directly
if __name__ == "__main__":
    solver = IKSolver()
    
    print("Testing Forward Kinematics:")
    print("="*50)
    
    # Test home position
    home_pos = solver.forward_kinematics([0, 0, 0, 0])
    print(f"Home position [0,0,0,0]: {home_pos}")
    
    # Test various configurations
    test_configs = [
        [0, 0.5, 0, 0],
        [0, 0, 0.5, 0],
        [1.57, 0, 0, 0],
        [0, 0.5, 0.5, 0],
    ]
    
    for config in test_configs:
        pos = solver.forward_kinematics(config)
        print(f"Config {config}: {pos}")
    
    print("\nWorkspace bounds:")
    solver.print_workspace_bounds()
    
    print("\nTesting Inverse Kinematics:")
    print("="*50)
    
    # Test IK with a known reachable position
    target = home_pos + np.array([0.05, 0.05, 0.05])
    print(f"Target position: {target}")
    
    solution = solver.inverse_kinematics(target)
    if solution is not None:
        result_pos = solver.forward_kinematics(solution)
        print(f"Solution: {solution}")
        print(f"Achieved position: {result_pos}")
        print(f"Error: {np.linalg.norm(result_pos - target)*1000:.2f}mm")
