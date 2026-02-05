import os
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

class DumbbellSpawner(Node):
    def __init__(self) -> None:
        super().__init__('dumbbell_spawner')

        # 1. Define Color Map (RGBA)
        self.color_map = {
            'red':    '1.0 0.0 0.0 1.0',
            'blue':   '0.0 0.0 1.0 1.0',
            'green':  '0.0 1.0 0.0 1.0',
            'yellow': '1.0 1.0 0.0 1.0'
        }

        # 2. Load the URDF
        # We explicitly look for the installed file
        pkg_share = get_package_share_directory('sortabot')
        self.template_path = os.path.join(pkg_share, 'urdf', 'final_dumbbell.urdf')

        self.template_content = ""
        if not os.path.exists(self.template_path):
            # CRITICAL ERROR LOG
            self.get_logger().error(f'!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            self.get_logger().error(f'FILE NOT FOUND: {self.template_path}')
            self.get_logger().error(f'You must run "colcon build" to install the .urdf file!')
            self.get_logger().error(f'!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
        else:
            with open(self.template_path, 'r') as f:
                self.template_content = f.read()
            self.get_logger().info(f'SUCCESS: Loaded URDF from {self.template_path}')

        # 3. Subscribe
        self.subscription = self.create_subscription(
            String, 'dumbbell_info', self.dumbbell_callback, 10
        )
        self.spawn_count = 0
        self.get_logger().info('Dumbbell Spawner Ready.')

    def dumbbell_callback(self, msg: String) -> None:
        if not self.template_content:
            self.get_logger().error("CANNOT SPAWN: Template content is empty/missing.")
            return

        try:
            color_name, weight_str = msg.data.split(',')
        except ValueError:
            return

        self.spawn_count += 1
        entity_name = f'dumbbell_{self.spawn_count}_{color_name}'
        rgba_values = self.color_map.get(color_name, '1.0 1.0 1.0 1.0')

        # Replace Placeholder
        robot_xml = self.template_content.replace('COLOR_RGBA', rgba_values)

        x_pos = 0.5 * self.spawn_count
        
        self.get_logger().info(f'SPAWNING {entity_name}...')

        # --- THE FIX: REMOVED "-world" ARGUMENT ---
        # This allows it to spawn in the current world, whatever its name is.
        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-string', robot_xml,
            '-name', entity_name,
            '-x', str(x_pos),
            '-y', '0.0',
            '-z', '0.5'
        ]

        subprocess.Popen(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = DumbbellSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
