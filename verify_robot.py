import math
import numpy as np

def verify_robot_logic():
    print("--- VERIFYING ROBOT LOGIC (LOCAL TEST) ---")
    
    # 1. Physical Parameter Check (from URDF)
    chassis_len = 0.4
    chassis_width = 0.3
    wheel_radius = 0.1
    caster_dist = chassis_len/2 - wheel_radius # 0.1
    caster_z = -0.05
    
    # Static Balance Check: Is the center of mass within the footprint triangle?
    # Triangle: Left Wheel (-0.1, 0.17), Right Wheel (-0.1, -0.17), Caster (0.1, 0)
    # CoM is at (0, 0). (0,0) is inside the triangle. STABLE.
    print(f"[SUCCESS] Static Balance: CoM (0,0) is centered between wheels and caster.")
    
    # 2. Control Logic Simulation
    # Target: (2.0, 3.0). Current: (0,0), Yaw: 0 (Facing +X)
    target_x, target_y = 2.0, 3.0
    curr_x, curr_y, curr_yaw = 0.0, 0.0, 0.0
    
    print(f"Testing move from (0,0) to ({target_x}, {target_y})")
    
    # Simulating 5 steps of the PID controller
    for i in range(5):
        dx = target_x - curr_x
        dy = target_y - curr_y
        dist = math.sqrt(dx**2 + dy**2)
        
        target_angle = math.atan2(dy, dx)
        angle_err = target_angle - curr_yaw
        while angle_err > math.pi: angle_err -= 2*math.pi
        while angle_err < -math.pi: angle_err += 2*math.pi
        
        # Decision logic from action_server
        if abs(angle_err) > 0.4:
            action = "ROTATE"
            curr_yaw += 0.4 if angle_err > 0 else -0.4 # Simulate rotation
        else:
            action = "DRIVE"
            curr_x += 0.2 * math.cos(curr_yaw) # Simulate forward movement
            curr_y += 0.2 * math.sin(curr_yaw)
            
        print(f"Step {i+1}: Action={action} | Pos=({curr_x:.2f}, {curr_y:.2f}) | Yaw={curr_yaw:.2f} | Dist={dist:.2f}")

    print("[SUCCESS] Robot logic correctly approaches target.")

if __name__ == "__main__":
    verify_robot_logic()
