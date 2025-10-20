#!/usr/bin/env python3

"""
DQN Agent ROS2 Node for TurtleBot3 Navigation
Implements Deep Q-Learning for autonomous navigation
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from collections import deque
import random
import math


class DQNNetwork(nn.Module):
    """Deep Q-Network for navigation"""
    
    def __init__(self, state_size, action_size, hidden_size=128):
        super(DQNNetwork, self).__init__()
        self.fc1 = nn.Linear(state_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.fc3 = nn.Linear(hidden_size, action_size)
        
    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        return self.fc3(x)


class ReplayBuffer:
    """Experience replay buffer for DQN"""
    
    def __init__(self, capacity=10000):
        self.buffer = deque(maxlen=capacity)
    
    def push(self, state, action, reward, next_state, done):
        self.buffer.append((state, action, reward, next_state, done))
    
    def sample(self, batch_size):
        return random.sample(self.buffer, batch_size)
    
    def __len__(self):
        return len(self.buffer)


class DQNNavigationNode(Node):
    """ROS2 Node implementing DQN-based navigation"""
    
    def __init__(self):
        super().__init__('dqn_navigation_node')
        
        # Declare parameters
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('learning_rate', 0.001)
        self.declare_parameter('gamma', 0.99)
        self.declare_parameter('epsilon_start', 1.0)
        self.declare_parameter('epsilon_end', 0.01)
        self.declare_parameter('epsilon_decay', 0.995)
        self.declare_parameter('batch_size', 64)
        self.declare_parameter('model_path', '')
        self.declare_parameter('training_mode', False)
        
        # Get parameters
        self.learning_rate = self.get_parameter('learning_rate').value
        self.gamma = self.get_parameter('gamma').value
        self.epsilon = self.get_parameter('epsilon_start').value
        self.epsilon_end = self.get_parameter('epsilon_end').value
        self.epsilon_decay = self.get_parameter('epsilon_decay').value
        self.batch_size = self.get_parameter('batch_size').value
        self.model_path = self.get_parameter('model_path').value
        self.training_mode = self.get_parameter('training_mode').value
        
        # Action space: [forward, left, right, stop]
        self.actions = [
            [0.22, 0.0],   # Forward
            [0.1, 1.0],    # Turn left
            [0.1, -1.0],   # Turn right
            [0.0, 0.0]     # Stop
        ]
        self.action_size = len(self.actions)
        
        # State space: reduced LiDAR scans (24 points) + goal distance + goal angle
        self.lidar_points = 24
        self.state_size = self.lidar_points + 2
        
        # Initialize DQN
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.policy_net = DQNNetwork(self.state_size, self.action_size).to(self.device)
        self.target_net = DQNNetwork(self.state_size, self.action_size).to(self.device)
        self.target_net.load_state_dict(self.policy_net.state_dict())
        self.target_net.eval()
        
        self.optimizer = optim.Adam(self.policy_net.parameters(), lr=self.learning_rate)
        self.replay_buffer = ReplayBuffer()
        
        # Load model if path provided
        if self.model_path:
            self.load_model(self.model_path)
        
        # ROS2 subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # ROS2 publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # State variables
        self.scan_data = None
        self.current_position = None
        self.current_orientation = None
        self.goal_position = None
        self.last_state = None
        self.last_action = None
        self.episode_reward = 0.0
        self.steps = 0
        
        self.get_logger().info(f'DQN Navigation Node started (Training: {self.training_mode})')
        self.get_logger().info(f'Using device: {self.device}')
    
    def scan_callback(self, msg):
        """Process LiDAR scan data"""
        self.scan_data = np.array(msg.ranges)
        # Replace inf values with max range
        self.scan_data[np.isinf(self.scan_data)] = msg.range_max
    
    def odom_callback(self, msg):
        """Process odometry data"""
        self.current_position = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation
    
    def goal_callback(self, msg):
        """Process goal pose"""
        self.goal_position = msg.pose.position
        self.get_logger().info(f'New goal received: ({self.goal_position.x:.2f}, {self.goal_position.y:.2f})')
    
    def get_state(self):
        """Construct state vector from sensor data"""
        if self.scan_data is None or self.current_position is None or self.goal_position is None:
            return None
        
        # Reduce LiDAR scan to fixed number of points
        scan_step = len(self.scan_data) // self.lidar_points
        reduced_scan = self.scan_data[::scan_step][:self.lidar_points]
        
        # Normalize scan data
        normalized_scan = reduced_scan / 3.5  # Max range
        
        # Calculate goal distance and angle
        goal_distance = self.calculate_distance_to_goal()
        goal_angle = self.calculate_angle_to_goal()
        
        # Construct state vector
        state = np.concatenate([
            normalized_scan,
            [goal_distance / 5.0],  # Normalize distance
            [goal_angle / np.pi]    # Normalize angle
        ])
        
        return state
    
    def calculate_distance_to_goal(self):
        """Calculate Euclidean distance to goal"""
        if self.goal_position is None or self.current_position is None:
            return 0.0
        
        dx = self.goal_position.x - self.current_position.x
        dy = self.goal_position.y - self.current_position.y
        return math.sqrt(dx**2 + dy**2)
    
    def calculate_angle_to_goal(self):
        """Calculate angle to goal relative to robot orientation"""
        if self.goal_position is None or self.current_position is None:
            return 0.0
        
        dx = self.goal_position.x - self.current_position.x
        dy = self.goal_position.y - self.current_position.y
        goal_angle = math.atan2(dy, dx)
        
        # Get robot's current yaw
        quat = self.current_orientation
        robot_yaw = math.atan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y**2 + quat.z**2)
        )
        
        # Calculate relative angle
        angle_diff = goal_angle - robot_yaw
        # Normalize to [-pi, pi]
        while angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        while angle_diff < -np.pi:
            angle_diff += 2 * np.pi
        
        return angle_diff
    
    def select_action(self, state):
        """Select action using epsilon-greedy policy"""
        if self.training_mode and random.random() < self.epsilon:
            return random.randint(0, self.action_size - 1)
        
        with torch.no_grad():
            state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
            q_values = self.policy_net(state_tensor)
            return q_values.argmax().item()
    
    def calculate_reward(self, state):
        """Calculate reward based on current state"""
        if state is None:
            return 0.0
        
        reward = 0.0
        
        # Distance-based reward
        distance = self.calculate_distance_to_goal()
        if distance < 0.2:
            reward += 100.0  # Goal reached
            self.get_logger().info('Goal reached!')
        else:
            reward -= distance * 0.5  # Penalty for being far from goal
        
        # Collision penalty
        min_scan = np.min(state[:self.lidar_points]) * 3.5  # Denormalize
        if min_scan < 0.18:
            reward -= 50.0
            self.get_logger().warn('Collision detected!')
        
        # Forward progress reward
        angle_to_goal = abs(state[-1] * np.pi)  # Denormalize
        if angle_to_goal < 0.5:
            reward += 1.0
        
        return reward
    
    def train_step(self):
        """Perform one training step"""
        if len(self.replay_buffer) < self.batch_size:
            return
        
        batch = self.replay_buffer.sample(self.batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)
        
        states = torch.FloatTensor(np.array(states)).to(self.device)
        actions = torch.LongTensor(actions).to(self.device)
        rewards = torch.FloatTensor(rewards).to(self.device)
        next_states = torch.FloatTensor(np.array(next_states)).to(self.device)
        dones = torch.FloatTensor(dones).to(self.device)
        
        # Current Q values
        current_q = self.policy_net(states).gather(1, actions.unsqueeze(1))
        
        # Next Q values
        next_q = self.target_net(next_states).max(1)[0].detach()
        target_q = rewards + (1 - dones) * self.gamma * next_q
        
        # Compute loss
        loss = nn.MSELoss()(current_q.squeeze(), target_q)
        
        # Optimize
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        
        # Update epsilon
        self.epsilon = max(self.epsilon_end, self.epsilon * self.epsilon_decay)
    
    def control_loop(self):
        """Main control loop"""
        state = self.get_state()
        
        if state is None:
            return
        
        # Select and execute action
        action_idx = self.select_action(state)
        action = self.actions[action_idx]
        
        cmd_vel = Twist()
        cmd_vel.linear.x = action[0]
        cmd_vel.angular.z = action[1]
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Calculate reward
        reward = self.calculate_reward(state)
        self.episode_reward += reward
        
        # Store experience for training
        if self.training_mode and self.last_state is not None:
            done = reward == 100.0 or reward <= -50.0
            self.replay_buffer.push(self.last_state, self.last_action, reward, state, done)
            self.train_step()
            
            if done:
                self.get_logger().info(f'Episode finished. Total reward: {self.episode_reward:.2f}')
                self.episode_reward = 0.0
        
        self.last_state = state
        self.last_action = action_idx
        self.steps += 1
        
        # Update target network periodically
        if self.steps % 1000 == 0:
            self.target_net.load_state_dict(self.policy_net.state_dict())
            if self.training_mode:
                self.save_model()
    
    def save_model(self, path=None):
        """Save model weights"""
        save_path = path or 'dqn_model.pth'
        torch.save({
            'policy_net': self.policy_net.state_dict(),
            'target_net': self.target_net.state_dict(),
            'optimizer': self.optimizer.state_dict(),
            'epsilon': self.epsilon
        }, save_path)
        self.get_logger().info(f'Model saved to {save_path}')
    
    def load_model(self, path):
        """Load model weights"""
        try:
            checkpoint = torch.load(path, map_location=self.device)
            self.policy_net.load_state_dict(checkpoint['policy_net'])
            self.target_net.load_state_dict(checkpoint['target_net'])
            self.optimizer.load_state_dict(checkpoint['optimizer'])
            self.epsilon = checkpoint.get('epsilon', self.epsilon_end)
            self.get_logger().info(f'Model loaded from {path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DQNNavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.training_mode:
            node.save_model()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

