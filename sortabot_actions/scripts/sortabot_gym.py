#!/usr/bin/env python3
import gymnasium as gym
from gymnasium import spaces
import numpy as np
import time

class SortabotGymEnv(gym.Env):
    """
    Enhanced Gymnasium Environment for Sort-a-bot.
    supports both 'real' (ROS2) mode and 'fast' (Internal Sim) mode.
    """
    def __init__(self, mode='fast', ros_node=None):
        super(SortabotGymEnv, self).__init__()
        self.mode = mode
        self.node = ros_node
        
        # Grid boundaries
        self.min_xy = -7.0
        self.max_xy = 7.0
        
        # Action Space: Discrete for faster convergence
        # 0: North, 1: South, 2: East, 3: West, 4: Stop
        self.action_space = spaces.Discrete(5)

        # Observation Space: [robot_x, robot_y, target_x, target_y]
        self.observation_space = spaces.Box(
            low=np.array([self.min_xy, self.min_xy, self.min_xy, self.min_xy]),
            high=np.array([self.max_xy, self.max_xy, self.max_xy, self.max_xy]),
            dtype=np.float32
        )

        self.robot_pos = np.array([0.0, 0.0])
        self.target_pos = np.array([5.0, 5.0])
        self.steps = 0
        self.max_steps = 100

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.steps = 0
        
        # Randomize positions for robust training
        self.robot_pos = np.random.uniform(self.min_xy, self.max_xy, size=2)
        self.target_pos = np.random.uniform(self.min_xy, self.max_xy, size=2)
        
        observation = np.concatenate([self.robot_pos, self.target_pos]).astype(np.float32)
        return observation, {}

    def step(self, action):
        self.steps += 1
        
        # Movement logic (Fast Sim Mode)
        step_deg = 0.5
        if action == 0: self.robot_pos[1] += step_deg # North
        elif action == 1: self.robot_pos[1] -= step_deg # South
        elif action == 2: self.robot_pos[0] += step_deg # East
        elif action == 3: self.robot_pos[0] -= step_deg # West
        
        # Boundary clipping
        self.robot_pos = np.clip(self.robot_pos, self.min_xy, self.max_xy)
        
        # Calculate Distance
        dist = np.linalg.norm(self.robot_pos - self.target_pos)
        
        # Reward Function (Critical for Fast Training)
        # 1. Negative distance (Dense reward)
        reward = -dist * 0.1 
        
        # 2. Big reward for reaching target
        terminated = False
        if dist < 0.5:
            reward += 50.0
            terminated = True
            
        # 3. Penalty for taking too long
        reward -= 0.01 
        
        truncated = self.steps >= self.max_steps
        
        observation = np.concatenate([self.robot_pos, self.target_pos]).astype(np.float32)
        return observation, reward, terminated, truncated, {}

    def render(self):
        # Console output for visual feedback
        print(f"Robot: {self.robot_pos} | Target: {self.target_pos} | Dist: {np.linalg.norm(self.robot_pos - self.target_pos):.2f}")
