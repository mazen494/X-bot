#!/usr/bin/env python3
"""
Barista Robot Controller
Executes drink recipes based on customer orders.
8 Stations: home, cup_dispenser, ice_dispenser, drink1-4_dispenser, serving_station
4 Drinks: Drink1, Drink2, Drink3, Drink4
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import yaml
import os
import time


class BaristaRobot(Node):
    def __init__(self):
        super().__init__('barista_robot')
        
        # Load recipes from YAML
        self.load_config()
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10
        )
        self.status_pub = self.create_publisher(
            String, '/barista/status', 10
        )
        
        # Subscribers
        self.order_sub = self.create_subscription(
            String, '/barista/order', self.order_callback, 10
        )
        
        # Current state
        self.current_position = self.stations['home']['joints'].copy()
        self.is_busy = False
        self.order_queue = []
        
        # Joint names
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
        # Publish initial state
        self.publish_joint_state(self.current_position)
        
        # Print startup info
        self.print_startup_info()
    
    def load_config(self):
        """Load configuration from YAML file."""
        config_paths = [
            '/home/mma/x-bot/src/Assem6/config/recipes.yaml',
            os.path.join(os.path.dirname(__file__), '../config/recipes.yaml'),
        ]
        
        config_loaded = False
        for path in config_paths:
            if os.path.exists(path):
                with open(path, 'r') as f:
                    config = yaml.safe_load(f)
                self.stations = config.get('stations', {})
                self.recipes = config.get('recipes', {})
                self.motion_settings = config.get('motion_settings', {})
                self.get_logger().info(f"✓ Loaded config: {path}")
                config_loaded = True
                break
        
        if not config_loaded:
            self.get_logger().warn("Config not found, using defaults")
            self.load_defaults()
    
    def load_defaults(self):
        """Load default configuration."""
        self.stations = {
            'home': {'joints': [0.0, 0.0, 0.0, 0.0], 'action': 'idle', 'action_duration': 0.5},
            'cup_dispenser': {'joints': [-1.5794, 0.7113, -1.57, 0.781], 'action': 'dispense_cup', 'action_duration': 2.0},
            'ice_dispenser': {'joints': [-1.57, 0.8414, -1.4312, 0.781], 'action': 'dispense_ice', 'action_duration': 3.0},
            'drink1_dispenser': {'joints': [-0.8157, 0.6852, -1.57, 0.538], 'action': 'dispense_drink1', 'action_duration': 4.0},
            'drink2_dispenser': {'joints': [0.0521, 0.8848, -1.57, 0.8505], 'action': 'dispense_drink2', 'action_duration': 4.0},
            'drink3_dispenser': {'joints': [0.0, 0.9541, -1.5006, 0.8505], 'action': 'dispense_drink3', 'action_duration': 4.0},
            'drink4_dispenser': {'joints': [1.51, 0.9541, -1.5006, 0.8505], 'action': 'dispense_drink4', 'action_duration': 4.0},
            'serving_station': {'joints': [1.57, 0.6679, -0.8587, 0.0], 'action': 'serve_drink', 'action_duration': 2.0},
        }
        
        self.recipes = {
            'Drink1': {'name': 'Drink1', 'sequence': [
                {'station': 'home'}, {'station': 'cup_dispenser'}, {'station': 'ice_dispenser'},
                {'station': 'drink1_dispenser'}, {'station': 'serving_station'}, {'station': 'home'}
            ]},
            'Drink2': {'name': 'Drink2', 'sequence': [
                {'station': 'home'}, {'station': 'cup_dispenser'}, {'station': 'ice_dispenser'},
                {'station': 'drink2_dispenser'}, {'station': 'serving_station'}, {'station': 'home'}
            ]},
            'Drink3': {'name': 'Drink3', 'sequence': [
                {'station': 'home'}, {'station': 'cup_dispenser'}, {'station': 'ice_dispenser'},
                {'station': 'drink3_dispenser'}, {'station': 'serving_station'}, {'station': 'home'}
            ]},
            'Drink4': {'name': 'Drink4', 'sequence': [
                {'station': 'home'}, {'station': 'cup_dispenser'}, {'station': 'ice_dispenser'},
                {'station': 'drink4_dispenser'}, {'station': 'serving_station'}, {'station': 'home'}
            ]},
        }
        
        self.motion_settings = {'default_move_duration': 2.0, 'pause_at_station': 0.5, 'interpolation_rate': 50}
    
    def print_startup_info(self):
        """Print startup information."""
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info("🤖 BARISTA ROBOT SYSTEM")
        self.get_logger().info("=" * 60)
        self.get_logger().info("")
        self.get_logger().info("📍 STATIONS (8 positions):")
        for i, (name, data) in enumerate(self.stations.items(), 1):
            desc = data.get('description', name)
            self.get_logger().info(f"   {i}. {name}: {desc}")
        self.get_logger().info("")
        self.get_logger().info("🍹 AVAILABLE DRINKS:")
        for name, recipe in self.recipes.items():
            self.get_logger().info(f"   • {name}")
        self.get_logger().info("")
        self.get_logger().info("📡 TOPICS:")
        self.get_logger().info("   • Subscribe: /barista/order (String)")
        self.get_logger().info("   • Publish:   /barista/status (String)")
        self.get_logger().info("   • Publish:   /joint_states (JointState)")
        self.get_logger().info("")
        self.get_logger().info("💡 TO ORDER A DRINK:")
        self.get_logger().info("   ros2 topic pub --once /barista/order std_msgs/msg/String \"data: 'Drink1'\"")
        self.get_logger().info("=" * 60)
        self.get_logger().info("")
    
    def order_callback(self, msg):
        """Handle incoming drink orders."""
        drink_id = msg.data.strip()
        
        # Check if drink exists (case-insensitive search)
        matched_drink = None
        for key in self.recipes.keys():
            if key.lower() == drink_id.lower():
                matched_drink = key
                break
        
        if not matched_drink:
            self.get_logger().error(f"❌ Unknown drink: {drink_id}")
            self.get_logger().info(f"   Available: {list(self.recipes.keys())}")
            self.publish_status(f"Error: Unknown drink {drink_id}")
            return
        
        self.get_logger().info(f"📥 Order received: {matched_drink}")
        
        if self.is_busy:
            self.order_queue.append(matched_drink)
            self.get_logger().info(f"⏳ Added to queue (position: {len(self.order_queue)})")
            self.publish_status(f"Queued: {matched_drink} (position {len(self.order_queue)})")
        else:
            self.execute_recipe(matched_drink)
    
    def publish_joint_state(self, positions):
        """Publish joint state for visualization."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = list(positions)
        msg.velocity = [0.0] * 4
        msg.effort = [0.0] * 4
        self.joint_state_pub.publish(msg)
    
    def publish_status(self, status):
        """Publish robot status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def interpolate(self, start, end, steps=100):
        """Generate smooth trajectory with easing."""
        trajectory = []
        for i in range(steps + 1):
            t = i / steps
            # Smooth easing (ease-in-out)
            t = t * t * (3 - 2 * t)
            point = [s + t * (e - s) for s, e in zip(start, end)]
            trajectory.append(point)
        return trajectory
    
    def move_to_station(self, station_name):
        """Move robot smoothly to a station."""
        if station_name not in self.stations:
            self.get_logger().error(f"Unknown station: {station_name}")
            return False
        
        station = self.stations[station_name]
        target = station['joints']
        duration = self.motion_settings.get('default_move_duration', 2.0)
        rate = self.motion_settings.get('interpolation_rate', 50)
        
        self.get_logger().info(f"   🔄 Moving to: {station_name}")
        self.publish_status(f"Moving to {station_name}")
        
        # Generate smooth trajectory
        steps = int(duration * rate)
        trajectory = self.interpolate(self.current_position, target, steps)
        
        # Execute trajectory
        for positions in trajectory:
            self.publish_joint_state(positions)
            time.sleep(duration / steps)
        
        self.current_position = target.copy()
        
        # Perform action at station
        action = station.get('action', 'waiting')
        action_duration = station.get('action_duration', 0.5)
        
        if action_duration > 0:
            self.get_logger().info(f"   ⚙️  Action: {action} ({action_duration}s)")
            self.publish_status(f"Performing: {action}")
            time.sleep(action_duration)
        
        return True
    
    def execute_recipe(self, drink_id):
        """Execute a complete drink recipe."""
        self.is_busy = True
        recipe = self.recipes[drink_id]
        drink_name = recipe.get('name', drink_id)
        sequence = recipe['sequence']
        
        self.get_logger().info("")
        self.get_logger().info("╔" + "═" * 58 + "╗")
        self.get_logger().info(f"║  🍹 PREPARING: {drink_name:<42} ║")
        self.get_logger().info(f"║  📋 Steps: {len(sequence):<46} ║")
        self.get_logger().info("╚" + "═" * 58 + "╝")
        
        self.publish_status(f"Preparing {drink_name}")
        
        start_time = time.time()
        
        for i, step in enumerate(sequence):
            station = step['station']
            self.get_logger().info(f"\n   Step {i+1}/{len(sequence)}:")
            self.move_to_station(station)
        
        elapsed = time.time() - start_time
        
        self.get_logger().info("")
        self.get_logger().info("╔" + "═" * 58 + "╗")
        self.get_logger().info(f"║  ✅ {drink_name} READY!{' ' * (42 - len(drink_name))} ║")
        self.get_logger().info(f"║  ⏱️  Total time: {elapsed:.1f} seconds{' ' * 33} ║")
        self.get_logger().info("╚" + "═" * 58 + "╝")
        self.get_logger().info("")
        
        self.publish_status(f"{drink_name} ready! ({elapsed:.1f}s)")
        self.is_busy = False
        
        # Process next order in queue
        if self.order_queue:
            next_order = self.order_queue.pop(0)
            self.get_logger().info(f"📋 Processing next order: {next_order}")
            self.get_logger().info(f"   Remaining in queue: {len(self.order_queue)}")
            time.sleep(1.0)
            self.execute_recipe(next_order)


def main(args=None):
    rclpy.init(args=args)
    node = BaristaRobot()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\n🛑 Barista Robot shutting down...")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
