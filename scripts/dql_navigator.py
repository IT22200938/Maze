#!/usr/bin/env python3

"""
Deep Q-Learning Navigator for TurtleBot3 Maze Navigation
This implements a DQL agent for autonomous navigation in the maze environment.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from collections import deque
import random
import math

class DQN(nn.Module):
    """Deep Q-Network for navigation"""
    def __init__(self, state_size, action_size):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(state_size, 128)
        self.fc2 = nn.Linear(128, 128)
        self.fc3 = nn.Linear(128, 64)
        self.fc4 = nn.Linear(64, action_size)
        
    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = torch.relu(self.fc3(x))
        return self.fc4(x)

class DQLNavigator(Node):
    """ROS 2 Node for Deep Q-Learning Navigation"""
    
    def __init__(self):
        super().__init__('dql_navigator')
        
        # ROS 2 Publishers and Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # DQL Parameters
        self.state_size = 24  # 24 LiDAR readings (reduced from 360)
        self.action_size = 5  # Forward, Left, Right, Sharp Left, Sharp Right
        self.gamma = 0.99     # Discount factor
        self.epsilon = 1.0    # Exploration rate
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        self.learning_rate = 0.001
        self.batch_size = 64
        self.memory = deque(maxlen=2000)
        
        # Neural Networks
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = DQN(self.state_size, self.action_size).to(self.device)
        self.target_model = DQN(self.state_size, self.action_size).to(self.device)
        self.optimizer = optim.Adam(self.model.parameters(), lr=self.learning_rate)
        self.criterion = nn.MSELoss()
        
        # Initialize target model
        self.update_target_model()
        
        # State variables
        self.scan_data = None
        self.odom_data = None
        self.previous_position = None
        self.goal_position = (1.0, -1.0)  # Example goal (bottom-right)
        self.episode = 0
        self.step_count = 0
        self.total_reward = 0
        
        # Training parameters
        self.max_steps_per_episode = 500
        self.update_target_frequency = 10
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        
        self.get_logger().info('DQL Navigator initialized')
        self.get_logger().info(f'Using device: {self.device}')
        
    def scan_callback(self, msg):
        """Process LiDAR scan data"""
        # Reduce 360 readings to 24 (every 15 degrees)
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = msg.range_max
        ranges[np.isnan(ranges)] = 0.0
        
        # Sample every 15 degrees (360/24 = 15)
        self.scan_data = ranges[::15]
        
    def odom_callback(self, msg):
        """Process odometry data"""
        self.odom_data = msg
        
    def get_state(self):
        """Get current state from sensors"""
        if self.scan_data is None or self.odom_data is None:
            return None
            
        # Normalize LiDAR readings (0-3.5m range)
        normalized_scan = np.clip(self.scan_data / 3.5, 0, 1)
        
        # Get distance and angle to goal
        current_x = self.odom_data.pose.pose.position.x
        current_y = self.odom_data.pose.pose.position.y
        
        dx = self.goal_position[0] - current_x
        dy = self.goal_position[1] - current_y
        distance_to_goal = math.sqrt(dx**2 + dy**2)
        
        # Get robot orientation
        orientation = self.odom_data.pose.pose.orientation
        _, _, yaw = self.euler_from_quaternion(orientation)
        
        angle_to_goal = math.atan2(dy, dx) - yaw
        angle_to_goal = math.atan2(math.sin(angle_to_goal), math.cos(angle_to_goal))
        
        # Combine state: [24 LiDAR readings, distance_to_goal, angle_to_goal]
        # For simplicity, we'll use just LiDAR for now
        state = normalized_scan
        
        return state
        
    def get_action(self, state):
        """Select action using epsilon-greedy policy"""
        if random.random() < self.epsilon:
            # Explore: random action
            return random.randrange(self.action_size)
        else:
            # Exploit: use neural network
            state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
            with torch.no_grad():
                q_values = self.model(state_tensor)
            return q_values.argmax().item()
            
    def execute_action(self, action):
        """Execute action in environment"""
        cmd_vel = Twist()
        
        # Action mapping
        if action == 0:  # Forward
            cmd_vel.linear.x = 0.22
            cmd_vel.angular.z = 0.0
        elif action == 1:  # Forward + Left
            cmd_vel.linear.x = 0.15
            cmd_vel.angular.z = 0.5
        elif action == 2:  # Forward + Right
            cmd_vel.linear.x = 0.15
            cmd_vel.angular.z = -0.5
        elif action == 3:  # Sharp Left
            cmd_vel.linear.x = 0.05
            cmd_vel.angular.z = 1.0
        elif action == 4:  # Sharp Right
            cmd_vel.linear.x = 0.05
            cmd_vel.angular.z = -1.0
            
        self.cmd_vel_pub.publish(cmd_vel)
        
    def calculate_reward(self, state):
        """Calculate reward based on current state"""
        if state is None or self.odom_data is None:
            return 0
            
        reward = 0
        
        # 1. Collision penalty
        min_distance = np.min(self.scan_data)
        if min_distance < 0.2:
            reward -= 100  # Large penalty for collision
            self.get_logger().warn('Collision detected!')
            return reward
            
        # 2. Distance reward (encourage getting closer to goal)
        current_x = self.odom_data.pose.pose.position.x
        current_y = self.odom_data.pose.pose.position.y
        current_distance = math.sqrt(
            (self.goal_position[0] - current_x)**2 +
            (self.goal_position[1] - current_y)**2
        )
        
        if self.previous_position is not None:
            previous_distance = math.sqrt(
                (self.goal_position[0] - self.previous_position[0])**2 +
                (self.goal_position[1] - self.previous_position[1])**2
            )
            # Reward for moving closer to goal
            reward += (previous_distance - current_distance) * 10
            
        self.previous_position = (current_x, current_y)
        
        # 3. Goal reached reward
        if current_distance < 0.3:
            reward += 200
            self.get_logger().info('Goal reached!')
            
        # 4. Small penalty for each step (encourage efficiency)
        reward -= 0.1
        
        # 5. Encourage forward movement
        reward += 0.5
        
        return reward
        
    def remember(self, state, action, reward, next_state, done):
        """Store experience in replay memory"""
        self.memory.append((state, action, reward, next_state, done))
        
    def replay(self):
        """Train network using experience replay"""
        if len(self.memory) < self.batch_size:
            return
            
        # Sample random minibatch
        minibatch = random.sample(self.memory, self.batch_size)
        
        states = torch.FloatTensor([exp[0] for exp in minibatch]).to(self.device)
        actions = torch.LongTensor([exp[1] for exp in minibatch]).to(self.device)
        rewards = torch.FloatTensor([exp[2] for exp in minibatch]).to(self.device)
        next_states = torch.FloatTensor([exp[3] for exp in minibatch]).to(self.device)
        dones = torch.FloatTensor([exp[4] for exp in minibatch]).to(self.device)
        
        # Current Q values
        current_q_values = self.model(states).gather(1, actions.unsqueeze(1))
        
        # Next Q values from target network
        with torch.no_grad():
            next_q_values = self.target_model(next_states).max(1)[0]
            target_q_values = rewards + (1 - dones) * self.gamma * next_q_values
            
        # Compute loss and update
        loss = self.criterion(current_q_values.squeeze(), target_q_values)
        
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        
        # Decay epsilon
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay
            
        return loss.item()
        
    def update_target_model(self):
        """Update target network with current network weights"""
        self.target_model.load_state_dict(self.model.state_dict())
        
    def control_loop(self):
        """Main control loop"""
        # Get current state
        state = self.get_state()
        if state is None:
            return
            
        # Select and execute action
        action = self.get_action(state)
        self.execute_action(action)
        
        # Calculate reward
        reward = self.calculate_reward(state)
        self.total_reward += reward
        
        # Get next state
        next_state = self.get_state()
        
        # Check if episode is done
        done = False
        if reward <= -100:  # Collision
            done = True
        elif reward >= 200:  # Goal reached
            done = True
        elif self.step_count >= self.max_steps_per_episode:
            done = True
            
        # Store experience
        if next_state is not None:
            self.remember(state, action, reward, next_state, done)
            
        # Train network
        if len(self.memory) >= self.batch_size:
            loss = self.replay()
            if loss is not None and self.step_count % 10 == 0:
                self.get_logger().info(f'Loss: {loss:.4f}, Epsilon: {self.epsilon:.4f}')
        
        # Update counters
        self.step_count += 1
        
        # Handle episode end
        if done:
            self.get_logger().info(
                f'Episode {self.episode} finished. '
                f'Steps: {self.step_count}, '
                f'Total Reward: {self.total_reward:.2f}, '
                f'Epsilon: {self.epsilon:.4f}'
            )
            
            # Update target network periodically
            if self.episode % self.update_target_frequency == 0:
                self.update_target_model()
                self.get_logger().info('Target network updated')
            
            # Reset for next episode
            self.episode += 1
            self.step_count = 0
            self.total_reward = 0
            self.previous_position = None
            
            # Stop robot
            self.cmd_vel_pub.publish(Twist())
            
            # Save model periodically
            if self.episode % 50 == 0:
                self.save_model()
                
    def save_model(self):
        """Save trained model"""
        filename = f'dql_model_episode_{self.episode}.pth'
        torch.save({
            'model_state_dict': self.model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'episode': self.episode,
            'epsilon': self.epsilon
        }, filename)
        self.get_logger().info(f'Model saved: {filename}')
        
    def load_model(self, filename):
        """Load trained model"""
        checkpoint = torch.load(filename)
        self.model.load_state_dict(checkpoint['model_state_dict'])
        self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        self.episode = checkpoint['episode']
        self.epsilon = checkpoint['epsilon']
        self.update_target_model()
        self.get_logger().info(f'Model loaded: {filename}')
        
    @staticmethod
    def euler_from_quaternion(quaternion):
        """Convert quaternion to euler angles"""
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        
        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    
    navigator = DQLNavigator()
    
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Shutting down...')
        # Stop robot
        navigator.cmd_vel_pub.publish(Twist())
        # Save final model
        navigator.save_model()
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

