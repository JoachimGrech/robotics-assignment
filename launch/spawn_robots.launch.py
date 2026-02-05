import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'sortabot'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. Process the Robot Xacro (Convert to XML)
    xacro_file = os.path.join(pkg_share, 'urdf', 'sortabot.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()

    # 2. Get the Bins URDF
    bins_file = os.path.join(pkg_share, 'urdf', 'bins.urdf')
    with open(bins_file, 'r') as f:
        bins_desc = f.read()

    return LaunchDescription([
        # Node 1: Spawn Sortabot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-string', robot_desc,
                '-name', 'sortabot',
                '-x', '0.0', '-y', '0.0', '-z', '0.2'
            ],
            output='screen'
        ),
        
        # Node 2: Spawn Bins
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-string', bins_desc,
                '-name', 'sorting_bins',
                '-x', '2.5', '-y', '0.0', '-z', '0.0'  # Placed 2.5m away
            ],
            output='screen'
        ),
    ])
