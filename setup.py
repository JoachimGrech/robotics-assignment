from setuptools import setup
import os
from glob import glob

package_name = 'sortabot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # 1. Standard ROS 2 resources
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 2. Launch Files
        # We look in 'launch/', 'sortabot_description/launch/', and 'sortabot_description/'
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py') + 
            glob('sortabot_description/launch/*.launch.py') + 
            glob('sortabot_description/*.launch.py')),
        
        # 3. World Files
        # We look in 'worlds/', 'sortabot_description/worlds/', and 'sortabot_description/'
        (os.path.join('share', package_name, 'worlds'), 
            glob('worlds/*.world') + 
            glob('sortabot_description/worlds/*.world') + 
            glob('sortabot_description/*.world')),
        
        # 4. URDF and Xacro Files
        # We look in 'urdf/' and 'sortabot_description/urdf/'
        (os.path.join('share', package_name, 'urdf'), 
            glob('urdf/*.urdf') + 
            glob('urdf/*.xacro') +
            glob('sortabot_description/urdf/*.urdf') + 
            glob('sortabot_description/urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='You',
    author_email='you@example.com',
    description='Sortabot simulation package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bin_manager_gazebo = sortabot.bin_manager_gazebo:main',
            'dumbbell_driver    = sortabot.dumbbell_driver:main',
            'dumbbell_spawner   = sortabot.dumbbell_spawner:main',
            'childabot          = sortabot.childabot:main',
            'robot_controller   = sortabot.robot_controller:main',
            'world_manager      = sortabot.world_manager:main',
            'sortabot_action_server = sortabot.action_server:main',
        ],
    },
)
