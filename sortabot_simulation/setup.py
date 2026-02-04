from setuptools import setup

package_name = 'sortabot_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', ['worlds/sortabot.world']),
        ('share/' + package_name + '/models', [
            'models/sortabot.sdf',
            'models/childabot.sdf',
            'models/dumbbell.urdf.xacro'
        ]),
        ('share/' + package_name + '/launch', ['launch/sortabot.launch.py', 'launch/spawn_robots.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Joachim Grech',
    author_email='joachim.grech.23@um.edu.mt',
    description='Sortabot simulation package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'childabot = sortabot_simulation.childabot:main',
            'robot_controller = sortabot_simulation.robot_controller:main',
            'world_manager = sortabot_simulation.world_manager:main',
        ],
    },
)