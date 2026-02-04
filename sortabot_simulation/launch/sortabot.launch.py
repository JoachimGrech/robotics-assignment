import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Set software rendering BEFORE anything else (critical for VM)
    os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    
    pkg_simulation = get_package_share_directory('sortabot_simulation')
    pkg_description = get_package_share_directory('sortabot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    world_file = os.path.join(pkg_simulation, 'worlds', 'sortabot.world')
    robot_model_path = os.path.join(pkg_description, 'urdf', 'sortabot.urdf.xacro')

    # Gazebo Sim (Harmonic) launch
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # Set software rendering for Gazebo (VM performance)
    os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'

    # Robot State Publisher
    robot_description_content = ParameterValue(
        Command(['xacro ', robot_model_path]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # Spawn robot in Gazebo Sim
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'sortabot',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.2'
        ],
        output='screen'
    )

    # Bridge for topics (cmd_vel, odom, tf, joint_states)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])