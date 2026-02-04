#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sortabot_actions.action import GoTo
from nav_msgs.msg import Odometry
from stable_baselines3 import PPO
import numpy as np
import os

class RLBridgeNode(Node):
    def __init__(self):
        super().__init__('rl_bridge_node')
        
        # 1. Load the trained model
        model_path = os.path.join(os.getcwd(), 'ppo_sortabot_model.zip')
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model not found at {model_path}! Please run train_rl.py first.")
            return
            
        self.model = PPO.load(model_path)
        self.get_logger().info("RL Model loaded successfully!")

        # 2. Setup ROS interface
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.action_client = ActionClient(self, GoTo, 'go_to_location')
        
        # State variables
        self.current_pos = np.array([0.0, 0.0])
        self.target_pos = np.array([2.0, 3.0]) # Example target in the world
        self.last_action_time = 0

    def odom_callback(self, msg):
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        
        # Run RL inference every 2 seconds to avoid spamming
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_action_time > 2.0:
            self.run_inference()
            self.last_action_time = current_time

    def run_inference(self):
        # Prepare observation [robot_x, robot_y, target_x, target_y]
        obs = np.concatenate([self.current_pos, self.target_pos]).astype(np.float32)
        
        # Predict action
        action, _ = self.model.predict(obs, deterministic=True)
        
        # Map Discrete Action (0-3) to a small Delta movement or a fixed coordinate
        # To keep it simple, we just print the action for now or send a small relative goal
        # Mapping 0,1,2,3 to North, South, East, West relative to current pos
        move_dist = 0.5
        goal_x, goal_y = self.current_pos[0], self.current_pos[1]
        
        if action == 0: goal_y += move_dist
        elif action == 1: goal_y -= move_dist
        elif action == 2: goal_x += move_dist
        elif action == 3: goal_x -= move_dist
        else: return # Stop action

        self.send_ros_goal(goal_x, goal_y)

    def send_ros_goal(self, x, y):
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            return

        goal_msg = GoTo.Goal()
        goal_msg.x = float(x)
        goal_msg.y = float(y)
        
        self.get_logger().info(f"RL Decision: Target ({x:.2f}, {y:.2f})")
        self.action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RLBridgeNode()
    if hasattr(node, 'model'):
        rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
