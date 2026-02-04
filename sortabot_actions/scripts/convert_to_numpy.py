import torch
import numpy as np
import zipfile
import io
import os

def extract_weights(model_path, output_script):
    # Load the model state dict from the zip
    with zipfile.ZipFile(model_path, "r") as archive:
        with archive.open("policy.pth") as f:
            state_dict = torch.load(f, map_location=torch.device('cpu'))

    # Extract weights and biases for the actor (policy) network
    # For SB3 PPO MlpPolicy:
    # mlp_extractor.policy_net.0 (Linear), .2 (Linear)
    # action_net (Linear)
    
    weights = {}
    for key, value in state_dict.items():
        if "mlp_extractor.policy_net" in key or "action_net" in key:
            weights[key] = value.numpy()

    # Create the standalone numpy script
    script_content = f"""#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sortabot_actions.action import GoTo
from nav_msgs.msg import Odometry

# PRE-TRAINED WEIGHTS (Extracted)
W0 = {np.array2string(weights['mlp_extractor.policy_net.0.weight'], separator=',', precision=8, threshold=1000000)}
B0 = {np.array2string(weights['mlp_extractor.policy_net.0.bias'], separator=',', precision=8, threshold=1000000)}
W1 = {np.array2string(weights['mlp_extractor.policy_net.2.weight'], separator=',', precision=8, threshold=1000000)}
B1 = {np.array2string(weights['mlp_extractor.policy_net.2.bias'], separator=',', precision=8, threshold=1000000)}
W_ACT = {np.array2string(weights['action_net.weight'], separator=',', precision=8, threshold=1000000)}
B_ACT = {np.array2string(weights['action_net.bias'], separator=',', precision=8, threshold=1000000)}

def tanh(x):
    return np.tanh(x)

def predict(obs):
    # Layer 1
    x = np.dot(W0, obs) + B0
    x = tanh(x)
    # Layer 2
    x = np.dot(W1, x) + B1
    x = tanh(x)
    # Output Layer
    logits = np.dot(W_ACT, x) + B_ACT
    return np.argmax(logits)

class RLNumpyNode(Node):
    def __init__(self):
        super().__init__('rl_numpy_node')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.action_client = ActionClient(self, GoTo, 'go_to_location')
        self.current_pos = np.array([0.0, 0.0])
        self.target_pos = np.array([2.0, 3.0]) # Same target
        self.last_time = 0

    def odom_callback(self, msg):
        self.current_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_time > 2.0:
            obs = np.concatenate([self.current_pos, self.target_pos]).astype(np.float32)
            action = predict(obs)
            self.execute_action(action)
            self.last_time = now

    def execute_action(self, action):
        move_dist = 0.5
        goal_x, goal_y = self.current_pos[0], self.current_pos[1]
        if action == 0: goal_y += move_dist
        elif action == 1: goal_y -= move_dist
        elif action == 2: goal_x += move_dist
        elif action == 3: goal_x -= move_dist
        else: return # Stop

        if self.action_client.wait_for_server(timeout_sec=1.0):
            goal = GoTo.Goal()
            goal.x, goal.y = float(goal_x), float(goal_y)
            self.get_logger().info(f'RL Numpy Action: {{action}} -> Target ({{goal_x:.2f}}, {{goal_y:.2f}})')
            self.action_client.send_goal_async(goal)

def main():
    rclpy.init()
    rclpy.spin(RLNumpyNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
    with open(output_script, "w") as f:
        f.write(script_content)
    print(f"Standalone script created: {output_script}")

if __name__ == "__main__":
    extract_weights("ppo_sortabot_model.zip", "local_rl_inference.py")
