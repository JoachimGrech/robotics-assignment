#!/usr/bin/env python3
import os

# Dumbbell configurations: color, weight_per_side, x, y
dumbbells = [
    ("red", 4.0, 2.0, 3.0),      # Red light: 8kg total
    ("red", 8.0, -2.0, 5.0),     # Red heavy: 16kg total
    ("blue", 3.0, -3.0, 1.5),    # Blue light: 6kg total
    ("blue", 9.0, 4.0, -1.0),    # Blue heavy: 18kg total
    ("green", 5.0, 1.0, -4.0),   # Green light: 10kg total
    ("green", 10.0, -4.0, -3.0), # Green heavy: 20kg total
    ("yellow", 3.5, 5.0, 2.0),   # Yellow light: 7kg total
    ("yellow", 7.0, -1.0, -5.0), # Yellow heavy: 14kg total
]

# Read the XACRO template (now in models folder)
with open("models/dumbbell.urdf.xacro", "r") as f:
    xacro_template = f.read()

# Generate SDF for each dumbbell
sdf_models = []
for i, (color, weight, x, y) in enumerate(dumbbells):
    # Simple string replacement in XACRO template
    urdf_content = xacro_template
    
    # Replace parameters (assuming your XACRO uses $(arg weight) and $(arg color))
    urdf_content = urdf_content.replace('$(arg weight)', str(weight))
    urdf_content = urdf_content.replace('$(arg color)', color)
    
    # Also replace if using different syntax
    urdf_content = urdf_content.replace('${weight}', str(weight))
    urdf_content = urdf_content.replace('${color}', color)
    
    # Create SDF directly (simpler than converting URDF to SDF)
    color_rgb = {
        'red': '1 0 0',
        'blue': '0 0 1',
        'green': '0 1 0',
        'yellow': '1 1 0'
    }[color]
    
    sdf = f'''    <model name="dumbbell_{color}_{i}">
      <pose>{x} {y} 0.03 0 1.5708 0</pose>
      <link name="handle">
        <pose>0 0 0 0 1.5708 0</pose>
        <visual name="handle_visual">
          <geometry>
            <cylinder>
              <length>0.15</length>
              <radius>0.015</radius>
            </cylinder>
          </geometry>
          <material>
            <ambient>0 0 0 1</ambient>
            <diffuse>0.1 0.1 0.1 1</diffuse>
          </material>
        </visual>
        <collision name="handle_collision">
          <geometry>
            <cylinder>
              <length>0.15</length>
              <radius>0.015</radius>
            </cylinder>
          </geometry>
        </collision>
      </link>
      
      <link name="left_weight">
        <pose>-0.095 0 0 0 1.5708 0</pose>
        <visual name="left_visual">
          <geometry>
            <cylinder>
              <length>0.04</length>
              <radius>0.06</radius>
            </cylinder>
          </geometry>
          <material>
            <ambient>{color_rgb} 1</ambient>
            <diffuse>{color_rgb} 1</diffuse>
          </material>
        </visual>
        <collision name="left_collision">
          <geometry>
            <cylinder>
              <length>0.04</length>
              <radius>0.06</radius>
            </cylinder>
          </geometry>
        </collision>
        <inertial>
          <mass>{weight}</mass>
        </inertial>
      </link>
      
      <link name="right_weight">
        <pose>0.095 0 0 0 1.5708 0</pose>
        <visual name="right_visual">
          <geometry>
            <cylinder>
              <length>0.04</length>
              <radius>0.06</radius>
            </cylinder>
          </geometry>
          <material>
            <ambient>{color_rgb} 1</ambient>
            <diffuse>{color_rgb} 1</diffuse>
          </material>
        </visual>
        <collision name="right_collision">
          <geometry>
            <cylinder>
              <length>0.04</length>
              <radius>0.06</radius>
            </cylinder>
          </geometry>
        </collision>
        <inertial>
          <mass>{weight}</mass>
        </inertial>
      </link>
      
      <joint name="left_joint" type="fixed">
        <parent>handle</parent>
        <child>left_weight</child>
      </joint>
      
      <joint name="right_joint" type="fixed">
        <parent>handle</parent>
        <child>right_weight</child>
      </joint>
    </model>'''
    
    sdf_models.append(sdf)

# Read existing world file
world_path = "worlds/sortabot.world"
with open(world_path, "r") as f:
    world_content = f.read()

# Insert dumbbells before the closing </world> tag
if "</world>" in world_content:
    # Insert all dumbbell models
    dumbbell_section = "\n\n    <!-- DUMBBELLS (generated) -->\n" + "\n".join(sdf_models)
    
    # Find where to insert (before </world>)
    parts = world_content.split("</world>")
    if len(parts) == 2:
        new_world = parts[0] + dumbbell_section + "\n\n  </world>"
        
        # Write updated world file
        with open(world_path, "w") as f:
            f.write(new_world)
        
        print("Ã¢Å“â€œ World file updated with 8 dumbbells!")
        print("  Red light (8kg total) at (2.0, 3.0)")
        print("  Red heavy (16kg total) at (-2.0, 5.0)")
        print("  Blue light (6kg total) at (-3.0, 1.5)")
        print("  Blue heavy (18kg total) at (4.0, -1.0)")
        print("  Green light (10kg total) at (1.0, -4.0)")
        print("  Green heavy (20kg total) at (-4.0, -3.0)")
        print("  Yellow light (7kg total) at (5.0, 2.0)")
        print("  Yellow heavy (14kg total) at (-1.0, -5.0)")
    else:
        print("Error: Could not find </world> tag")
else:
    print("Error: </world> tag not found")
