#!/usr/bin/env python3

"""
Goal Clicker Node for RViz2 Interaction
Allows users to click on the map in RViz2 to set navigation goals
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String


class GoalClickerNode(Node):
    """Node that processes goal poses from RViz2"""
    
    def __init__(self):
        super().__init__('goal_clicker_node')
        
        # Declare parameters
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('nav_mode', 'nav2')  # 'nav2' or 'dqn'
        
        # Get parameters
        goal_topic = self.get_parameter('goal_topic').value
        self.nav_mode = self.get_parameter('nav_mode').value
        
        # Subscribe to RViz goal pose (from "2D Goal Pose" tool)
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # Subscribe to RViz initial pose (from "2D Pose Estimate" tool)
        self.init_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.init_pose_callback,
            10
        )
        
        # Publishers
        self.nav2_goal_pub = self.create_publisher(
            PoseStamped,
            '/navigate_to_pose/_action/send_goal',
            10
        )
        
        self.dqn_goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/navigation_status',
            10
        )
        
        self.get_logger().info(f'Goal Clicker Node started (Mode: {self.nav_mode})')
        self.get_logger().info('Use "2D Goal Pose" tool in RViz2 to set navigation goals')
    
    def goal_callback(self, msg):
        """Process goal pose from RViz2"""
        self.get_logger().info(
            f'Goal received: x={msg.pose.position.x:.2f}, '
            f'y={msg.pose.position.y:.2f}'
        )
        
        # Publish to appropriate navigation system
        if self.nav_mode == 'nav2':
            self.nav2_goal_pub.publish(msg)
            self.get_logger().info('Goal forwarded to Nav2')
        elif self.nav_mode == 'dqn':
            self.dqn_goal_pub.publish(msg)
            self.get_logger().info('Goal forwarded to DQN agent')
        
        # Publish status
        status_msg = String()
        status_msg.data = f'Navigating to ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        self.status_pub.publish(status_msg)
    
    def init_pose_callback(self, msg):
        """Process initial pose from RViz2"""
        self.get_logger().info(
            f'Initial pose set: x={msg.pose.pose.position.x:.2f}, '
            f'y={msg.pose.pose.position.y:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GoalClickerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

