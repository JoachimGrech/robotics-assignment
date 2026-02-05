import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'sortabot'
    pkg_share = get_package_share_directory(pkg_name)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # 1. FILE PATHS
    robot_xacro_path = os.path.join(pkg_share, 'urdf', 'sortabot.urdf.xacro')
    bins_urdf_path = os.path.join(pkg_share, 'urdf', 'bins.urdf')
    world_path = os.path.join(pkg_share, 'worlds', 'sortabot.world')

    # 2. PROCESS ROBOT DESCRIPTION (XACRO)
    robot_description_config = xacro.process_file(robot_xacro_path)
    robot_desc = robot_description_config.toxml()

    # 3. READ BINS DESCRIPTION (URDF)
    with open(bins_urdf_path, 'r') as f:
        bins_desc = f.read()

    # 4. DEFINE THE NODES
    return LaunchDescription([
        
        # A. SET ENVIRONMENT VARIABLE (Critical for Jazzy Plugins)
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', '/opt/ros/jazzy/lib'),
        
        # B. START GAZEBO SIMULATION
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': f'-r {world_path}'}.items(),
        ),

        # C. ROBOT: STATE PUBLISHER
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
        ),

        # D. ROBOT: SPAWN
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', 'robot_description', '-name', 'sortabot', '-z', '0.2'],
            output='screen'
        ),

        # E. BINS: STATE PUBLISHER (Remapped)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='bins_state_publisher',
            output='screen',
            remappings=[('/robot_description', '/bins_description')],
            parameters=[{'robot_description': bins_desc, 'use_sim_time': True}]
        ),

        # F. BINS: SPAWN
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-topic', '/bins_description', '-name', 'sorting_bins'],
            output='screen'
        ),

        # G. ROS-GAZEBO BRIDGE
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
                
                # Your existing bridges:
                '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                '/dumbbell_info@std_msgs/msg/String@gz.msgs.StringMsg',
                '/dumbbell_command@std_msgs/msg/String@gz.msgs.StringMsg',
                
                # Camera Bridge
                '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
            ],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),

        # H. BIN MANAGER
        Node(
            package=pkg_name,
            executable='bin_manager_gazebo',
            name='smart_bin_manager',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # I. DUMBBELL SPAWNER
        Node(
            package=pkg_name,
            executable='dumbbell_spawner',
            name='dumbbell_spawner',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # J. DUMBBELL DRIVER
        Node(
            package=pkg_name,
            executable='dumbbell_driver',
            name='dumbbell_driver',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # K. SORTABOT ACTION SERVER
        Node(
            package=pkg_name,
            executable='sortabot_action_server',
            name='sortabot_action_server',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
