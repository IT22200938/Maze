#!/usr/bin/env python3

"""
Performance Recorder Node
Logs navigation metrics for evaluation and comparison
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
import json
import math
from datetime import datetime
import os


class PerformanceRecorder(Node):
    """Node to record and evaluate navigation performance"""
    
    def __init__(self):
        super().__init__('performance_recorder')
        
        # Declare parameters
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('output_dir', './navigation_logs')
        self.declare_parameter('navigation_mode', 'nav2')
        
        # Get parameters
        self.output_dir = self.get_parameter('output_dir').value
        self.nav_mode = self.get_parameter('navigation_mode').value
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10)
        
        self.emergency_sub = self.create_subscription(
            Bool, '/emergency_stop', self.emergency_callback, 10)
        
        self.status_sub = self.create_subscription(
            String, '/navigation_status', self.status_callback, 10)
        
        # State variables
        self.current_session = None
        self.sessions = []
        self.start_time = None
        self.start_position = None
        self.current_position = None
        self.goal_position = None
        self.total_distance = 0.0
        self.last_position = None
        self.collision_count = 0
        self.path_length = 0.0
        
        self.get_logger().info(f'Performance Recorder started (Mode: {self.nav_mode})')
        self.get_logger().info(f'Logs will be saved to: {self.output_dir}')
    
    def odom_callback(self, msg):
        """Process odometry data"""
        self.current_position = msg.pose.pose.position
        
        # Calculate distance traveled
        if self.last_position is not None and self.current_session is not None:
            dx = self.current_position.x - self.last_position.x
            dy = self.current_position.y - self.last_position.y
            distance = math.sqrt(dx**2 + dy**2)
            self.total_distance += distance
        
        self.last_position = self.current_position
        
        # Check if goal is reached
        if self.goal_position is not None and self.current_session is not None:
            goal_distance = self.calculate_distance(
                self.current_position, self.goal_position
            )
            
            if goal_distance < 0.2:  # Goal reached
                self.finish_session(success=True)
    
    def goal_callback(self, msg):
        """Process new goal"""
        self.get_logger().info('New goal received, starting new session')
        
        # Finish previous session if exists
        if self.current_session is not None:
            self.finish_session(success=False)
        
        # Start new session
        self.start_session(msg.pose.position)
    
    def path_callback(self, msg):
        """Process planned path"""
        if len(msg.poses) > 1:
            # Calculate path length
            path_length = 0.0
            for i in range(len(msg.poses) - 1):
                p1 = msg.poses[i].pose.position
                p2 = msg.poses[i + 1].pose.position
                path_length += self.calculate_distance(p1, p2)
            
            self.path_length = path_length
            
            if self.current_session is not None:
                self.current_session['planned_path_length'] = path_length
    
    def emergency_callback(self, msg):
        """Process emergency stop events"""
        if msg.data and self.current_session is not None:
            self.collision_count += 1
            self.current_session['collision_count'] = self.collision_count
            self.get_logger().warn(f'Collision detected! Total: {self.collision_count}')
    
    def status_callback(self, msg):
        """Process navigation status"""
        if self.current_session is not None:
            self.current_session['status'] = msg.data
    
    def start_session(self, goal_position):
        """Start a new recording session"""
        self.start_time = self.get_clock().now()
        self.start_position = self.current_position
        self.goal_position = goal_position
        self.total_distance = 0.0
        self.collision_count = 0
        self.path_length = 0.0
        
        self.current_session = {
            'session_id': len(self.sessions) + 1,
            'navigation_mode': self.nav_mode,
            'start_time': datetime.now().isoformat(),
            'start_position': {
                'x': self.start_position.x if self.start_position else 0.0,
                'y': self.start_position.y if self.start_position else 0.0
            },
            'goal_position': {
                'x': goal_position.x,
                'y': goal_position.y
            },
            'status': 'in_progress'
        }
        
        self.get_logger().info(f'Session {self.current_session["session_id"]} started')
    
    def finish_session(self, success=True):
        """Finish current recording session"""
        if self.current_session is None:
            return
        
        end_time = self.get_clock().now()
        duration = (end_time - self.start_time).nanoseconds / 1e9  # Convert to seconds
        
        # Calculate straight-line distance to goal
        if self.start_position and self.goal_position:
            straight_distance = self.calculate_distance(
                self.start_position, self.goal_position
            )
        else:
            straight_distance = 0.0
        
        # Update session data
        self.current_session.update({
            'end_time': datetime.now().isoformat(),
            'duration_seconds': duration,
            'total_distance_traveled': self.total_distance,
            'straight_line_distance': straight_distance,
            'path_efficiency': (straight_distance / self.total_distance * 100) if self.total_distance > 0 else 0.0,
            'collision_count': self.collision_count,
            'success': success,
            'average_speed': self.total_distance / duration if duration > 0 else 0.0
        })
        
        # Add to sessions list
        self.sessions.append(self.current_session)
        
        # Log summary
        self.get_logger().info(
            f'Session {self.current_session["session_id"]} finished:\n'
            f'  Success: {success}\n'
            f'  Duration: {duration:.2f}s\n'
            f'  Distance: {self.total_distance:.2f}m\n'
            f'  Collisions: {self.collision_count}\n'
            f'  Path efficiency: {self.current_session["path_efficiency"]:.1f}%'
        )
        
        # Save to file
        self.save_session()
        
        # Reset for next session
        self.current_session = None
        self.start_time = None
        self.goal_position = None
    
    def save_session(self):
        """Save session data to JSON file"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(
            self.output_dir,
            f'session_{self.nav_mode}_{timestamp}.json'
        )
        
        with open(filename, 'w') as f:
            json.dump(self.current_session, f, indent=2)
        
        self.get_logger().info(f'Session data saved to {filename}')
        
        # Also update cumulative log
        cumulative_file = os.path.join(self.output_dir, f'all_sessions_{self.nav_mode}.json')
        with open(cumulative_file, 'w') as f:
            json.dump(self.sessions, f, indent=2)
    
    @staticmethod
    def calculate_distance(pos1, pos2):
        """Calculate Euclidean distance between two positions"""
        dx = pos2.x - pos1.x
        dy = pos2.y - pos1.y
        return math.sqrt(dx**2 + dy**2)


def main(args=None):
    rclpy.init(args=args)
    node = PerformanceRecorder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

