#!/usr/bin/env python3

"""
Training script for DQN agent
Handles environment setup and training episodes
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import random
import time
import argparse


class DQNTrainer(Node):
    """Node to manage DQN training episodes"""
    
    def __init__(self, num_episodes=100):
        super().__init__('dqn_trainer')
        
        self.num_episodes = num_episodes
        self.current_episode = 0
        
        # Publisher for goal poses
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Training parameters
        self.episode_timeout = 120.0  # seconds
        self.goal_positions = [
            (0.5, 0.5), (2.5, 0.5), (0.5, 2.5), (2.5, 2.5),
            (1.5, 1.5), (0.5, 1.5), (2.5, 1.5), (1.5, 0.5), (1.5, 2.5)
        ]
        
        self.get_logger().info(f'DQN Trainer initialized for {num_episodes} episodes')
    
    def publish_random_goal(self):
        """Publish a random goal position"""
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        
        # Select random position
        x, y = random.choice(self.goal_positions)
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        
        # Random orientation
        goal.pose.orientation.w = 1.0
        
        self.goal_pub.publish(goal)
        self.get_logger().info(f'Episode {self.current_episode}: Goal at ({x:.2f}, {y:.2f})')
    
    def run_training(self):
        """Run training episodes"""
        self.get_logger().info('Starting training...')
        
        for episode in range(self.num_episodes):
            self.current_episode = episode + 1
            self.get_logger().info(f'Starting episode {self.current_episode}/{self.num_episodes}')
            
            # Publish new goal
            time.sleep(1.0)  # Wait for system to stabilize
            self.publish_random_goal()
            
            # Wait for episode to complete
            start_time = time.time()
            while time.time() - start_time < self.episode_timeout:
                rclpy.spin_once(self, timeout_sec=1.0)
            
            self.get_logger().info(f'Episode {self.current_episode} completed')
        
        self.get_logger().info('Training completed!')


def main(args=None):
    parser = argparse.ArgumentParser(description='DQN Training Script')
    parser.add_argument('--episodes', type=int, default=100,
                        help='Number of training episodes')
    
    parsed_args = parser.parse_args()
    
    rclpy.init(args=args)
    trainer = DQNTrainer(num_episodes=parsed_args.episodes)
    
    try:
        trainer.run_training()
    except KeyboardInterrupt:
        trainer.get_logger().info('Training interrupted by user')
    finally:
        trainer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

