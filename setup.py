from setuptools import setup
import os
from glob import glob

package_name = 'sortabot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sortabot.launch.py', 'launch/spawn_robots.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/sortabot.world']),
        
        # This block installs all your robot description files
        ('share/' + package_name + '/urdf', [
            'urdf/bins.urdf', 
            'urdf/final_dumbbell.urdf',
            'urdf/materials.xacro',
            'urdf/sortabot.urdf.xacro',
            'urdf/sortabot_arms.xacro', 
            'urdf/sortabot_base.xacro', 
            'urdf/sortabot_sensors.xacro'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='You',
    author_email='you@example.com',
    description='Sortabot simulation package',
    license='Apache-2.0',
    entry_points={
    'console_scripts': [
        'dumbbell_driver = sortabot.dumbbell_driver:main',
        'dumbbell_spawner = sortabot.dumbbell_spawner:main',
        'childabot = sortabot.childabot:main',
        'robot_controller = sortabot.robot_controller:main',
        'world_manager = sortabot.world_manager:main',
        ],
    },
)
