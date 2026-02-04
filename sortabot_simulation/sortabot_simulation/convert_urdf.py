#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
import subprocess
import sys

def convert_xacro_to_sdf():
    pkg_share = get_package_share_directory('sortabot')
    
    # Input XACRO file
    xacro_file = os.path.join(pkg_share, 'urdf', 'sortabot.urdf.xacro')
    
    # Output SDF file
    model_dir = os.path.expanduser('~/.gz/sim/models/sortabot')
    os.makedirs(model_dir, exist_ok=True)
    sdf_file = os.path.join(model_dir, 'model.sdf')
    
    # First convert xacro to urdf
    urdf_content = subprocess.run(
        ['xacro', xacro_file],
        capture_output=True,
        text=True
    ).stdout
    
    # Then convert URDF to SDF using gz sdf
    # Write URDF to temp file
    temp_urdf = '/tmp/temp_urdf.urdf'
    with open(temp_urdf, 'w') as f:
        f.write(urdf_content)
    
    # Convert using gz sdf
    result = subprocess.run(
        ['gz', 'sdf', '-p', temp_urdf],
        capture_output=True,
        text=True
    )
    
    if result.returncode == 0:
        with open(sdf_file, 'w') as f:
            f.write(result.stdout)
        print(f"Successfully converted to SDF: {sdf_file}")
    else:
        print(f"Error converting to SDF: {result.stderr}")
        # Fallback: create simple SDF manually
        create_simple_sdf(sdf_file)

def create_simple_sdf(sdf_file):
    """Create a simple SDF if conversion fails"""
    simple_sdf = """<?xml version="1.0"?>
<sdf version="1.7">
  <model name="sortabot">
    <link name="base_link">
      <pose>0 0 0.25 0 0 0</pose>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.5</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.2 0.4 1.0 1.0</ambient>
          <diffuse>0.3 0.5 1.0 1.0</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.1</radius>
            <length>0.5</length>
          </cylinder>
        </geometry>
      </collision>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.1</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.1</iyy>
          <iyz>0</iyz>
          <izz>0.1</izz>
        </inertia>
      </inertial>
    </link>
  </model>
</sdf>"""
    
    with open(sdf_file, 'w') as f:
        f.write(simple_sdf)
    print(f"Created simple SDF: {sdf_file}")

if __name__ == '__main__':
    convert_xacro_to_sdf()