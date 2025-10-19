#!/usr/bin/env python3

"""
Performance Tracker for Navigation Methods
Tracks time, path length, and success rate for navigation experiments
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatusArray
import math
import time
import json
from datetime import datetime

class PerformanceTracker(Node):
    def __init__(self):
        super().__init__('performance_tracker')
        
        # Subscribers
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.goal_status_sub = self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status', 
            self.status_callback, 10)
        
        # Data storage
        self.current_experiment = {
            'method': 'Nav2',  # or 'DQL'
            'goal_location': '',
            'start_time': None,
            'end_time': None,
            'path_length': 0.0,
            'actual_distance': 0.0,
            'success': False,
            'collisions': 0
        }
        
        self.experiments = []
        self.tracking_active = False
        self.goal_received_time = None
        self.previous_position = None
        self.total_distance_traveled = 0.0
        
        self.get_logger().info('Performance Tracker initialized')
        self.get_logger().info('Waiting for navigation goal...')
        
    def path_callback(self, msg):
        """Calculate planned path length"""
        if not msg.poses:
            return
            
        path_length = 0.0
        for i in range(len(msg.poses) - 1):
            p1 = msg.poses[i].pose.position
            p2 = msg.poses[i + 1].pose.position
            
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            path_length += math.sqrt(dx**2 + dy**2)
        
        self.current_experiment['path_length'] = path_length
        
        # If this is the first path, start tracking
        if not self.tracking_active:
            self.start_tracking()
            
    def odom_callback(self, msg):
        """Track actual distance traveled"""
        if not self.tracking_active:
            return
            
        current_pos = msg.pose.pose.position
        
        if self.previous_position is not None:
            dx = current_pos.x - self.previous_position.x
            dy = current_pos.y - self.previous_position.y
            distance = math.sqrt(dx**2 + dy**2)
            self.total_distance_traveled += distance
            
        self.previous_position = current_pos
        
    def status_callback(self, msg):
        """Track goal status"""
        if not msg.status_list:
            return
            
        latest_status = msg.status_list[-1]
        status_code = latest_status.status
        
        # Status codes: 1=ACCEPTED, 2=EXECUTING, 4=SUCCEEDED, 5=ABORTED
        if status_code == 4:  # SUCCEEDED
            self.stop_tracking(success=True)
        elif status_code == 5:  # ABORTED
            self.stop_tracking(success=False)
            
    def start_tracking(self):
        """Start tracking a new experiment"""
        self.tracking_active = True
        self.current_experiment['start_time'] = time.time()
        self.total_distance_traveled = 0.0
        self.previous_position = None
        
        self.get_logger().info('Tracking started!')
        
    def stop_tracking(self, success):
        """Stop tracking and save results"""
        if not self.tracking_active:
            return
            
        self.tracking_active = False
        self.current_experiment['end_time'] = time.time()
        self.current_experiment['success'] = success
        self.current_experiment['actual_distance'] = self.total_distance_traveled
        
        # Calculate metrics
        duration = self.current_experiment['end_time'] - self.current_experiment['start_time']
        planned = self.current_experiment['path_length']
        actual = self.current_experiment['actual_distance']
        
        efficiency = (planned / actual * 100) if actual > 0 else 0
        
        # Log results
        self.get_logger().info('='*50)
        self.get_logger().info('Experiment Complete!')
        self.get_logger().info(f"Success: {success}")
        self.get_logger().info(f"Duration: {duration:.2f} seconds")
        self.get_logger().info(f"Planned Path Length: {planned:.2f} meters")
        self.get_logger().info(f"Actual Distance Traveled: {actual:.2f} meters")
        self.get_logger().info(f"Path Efficiency: {efficiency:.1f}%")
        self.get_logger().info('='*50)
        
        # Save experiment
        experiment_data = self.current_experiment.copy()
        experiment_data['duration'] = duration
        experiment_data['efficiency'] = efficiency
        experiment_data['timestamp'] = datetime.now().isoformat()
        
        self.experiments.append(experiment_data)
        self.save_results()
        
        # Reset for next experiment
        self.current_experiment = {
            'method': self.current_experiment['method'],
            'goal_location': '',
            'start_time': None,
            'end_time': None,
            'path_length': 0.0,
            'actual_distance': 0.0,
            'success': False,
            'collisions': 0
        }
        
    def save_results(self):
        """Save all results to JSON file"""
        filename = 'navigation_results.json'
        
        try:
            with open(filename, 'w') as f:
                json.dump(self.experiments, f, indent=2)
            self.get_logger().info(f'Results saved to {filename}')
        except Exception as e:
            self.get_logger().error(f'Failed to save results: {e}')
            
    def print_summary(self):
        """Print summary statistics"""
        if not self.experiments:
            self.get_logger().info('No experiments recorded yet')
            return
            
        total = len(self.experiments)
        successes = sum(1 for e in self.experiments if e['success'])
        
        avg_time = sum(e['duration'] for e in self.experiments) / total
        avg_distance = sum(e['actual_distance'] for e in self.experiments) / total
        avg_efficiency = sum(e['efficiency'] for e in self.experiments) / total
        
        self.get_logger().info('='*50)
        self.get_logger().info('SUMMARY STATISTICS')
        self.get_logger().info('='*50)
        self.get_logger().info(f"Total Experiments: {total}")
        self.get_logger().info(f"Successful: {successes} ({successes/total*100:.1f}%)")
        self.get_logger().info(f"Average Time: {avg_time:.2f} seconds")
        self.get_logger().info(f"Average Distance: {avg_distance:.2f} meters")
        self.get_logger().info(f"Average Efficiency: {avg_efficiency:.1f}%")
        self.get_logger().info('='*50)

def main(args=None):
    rclpy.init(args=args)
    
    tracker = PerformanceTracker()
    
    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        tracker.get_logger().info('Shutting down...')
        tracker.print_summary()
    finally:
        tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

