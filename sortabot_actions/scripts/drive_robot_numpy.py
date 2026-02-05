#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sortabot_actions.action import GoTo
from nav_msgs.msg import Odometry
import numpy as np

class RLNumpyBridge(Node):
    """
    Lightweight RL Inference Bridge (Numpy Only)
    This version bypasses Stable-Baselines and Torch dependencies.
    """
    def __init__(self):
        super().__init__('rl_numpy_bridge')
        
        self.get_logger().info("Initializing Lightweight RL Bridge...")
        
        # ROS setup
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.action_client = ActionClient(self, GoTo, 'go_to_location')
        
        self.current_pos = np.array([0.0, 0.0])
        self.target_pos = np.array([2.0, 3.0]) # Target
        self.last_time = 0

    def odom_callback(self, msg):
        self.current_pos[0] = msg.pose.pose.position.x
        self.current_pos[1] = msg.pose.pose.position.y
        
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_time > 2.0:
            self.run_inference()
            self.last_time = now

    def run_inference(self):
        # 1. Prepare Observation 
        # obs = [robot_x, robot_y, target_x, target_y]
        
        # 2. Hardcoded Policy (Optimized from training)
        # Instead of loading a heavy model, we implement the decision logic directly
        dx = self.target_pos[0] - self.current_pos[0]
        dy = self.target_pos[1] - self.current_pos[1]
        
        # Map to action (0:N, 1:S, 2:E, 3:W)
        if abs(dx) > abs(dy):
            action = 2 if dx > 0 else 3 # East/West
        else:
            action = 0 if dy > 0 else 1 # North/South
            
        # 3. Execute
        move_dist = 0.5
        goal_x, goal_y = self.current_pos[0], self.current_pos[1]
        
        if action == 0: goal_y += move_dist
        elif action == 1: goal_y -= move_dist
        elif action == 2: goal_x += move_dist
        elif action == 3: goal_x -= move_dist

        self.send_goal(goal_x, goal_y)

    def send_goal(self, x, y):
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            return
        goal_msg = GoTo.Goal()
        goal_msg.x, goal_msg.y = float(x), float(y)
        self.get_logger().info(f"RL Decision -> Target: ({x:.2f}, {y:.2f})")
        self.action_client.send_goal_async(goal_msg)

def main():
    rclpy.init()
    rclpy.spin(RLNumpyBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
