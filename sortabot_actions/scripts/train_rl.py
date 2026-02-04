#!/usr/bin/env python3
import os
from stable_baselines3 import PPO
from sortabot_gym import SortabotGymEnv
import time

def train():
    # 1. Create Environment
    print("Initializing Environment...")
    env = SortabotGymEnv(mode='fast')

    # 2. Define Model (PPO is great for this task)
    # Using a small MLP for speed
    model = PPO(
        "MlpPolicy", 
        env, 
        verbose=1, 
        learning_rate=0.001,
        n_steps=1024,
        batch_size=64,
        tensorboard_log="./ppo_sortabot_log/"
    )

    # 3. Train
    print("Starting Training (Fast Mode)...")
    start_time = time.time()
    # 50,000 steps usually converge for this simple task in ~30 seconds
    model.learn(total_timesteps=50000)
    end_time = time.time()
    
    print(f"Training Complete! Time taken: {end_time - start_time:.2f} seconds")

    # 4. Save Model
    model.save("ppo_sortabot_model")
    print("Model saved as 'ppo_sortabot_model.zip'")

    # 5. Test the trained model
    print("\nStarting Testing...")
    obs, _ = env.reset()
    for _ in range(20):
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = env.step(action)
        env.render()
        if terminated or truncated:
            obs, _ = env.reset()
            print("--- New Episode ---")

if __name__ == "__main__":
    train()
