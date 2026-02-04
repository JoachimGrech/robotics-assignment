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

        # 2. Load the "final_dumbbell.urdf" which now acts as our template
        pkg_share = get_package_share_directory('sortabot')
        self.template_path = os.path.join(pkg_share, 'urdf', 'final_dumbbell.urdf')

        if not os.path.exists(self.template_path):
            self.get_logger().error(f'Template not found: {self.template_path}')
            self.template_content = ""
        else:
            with open(self.template_path, 'r') as f:
                self.template_content = f.read()

        # 3. Subscribe to the driver
        self.subscription = self.create_subscription(
            String, 'dumbbell_info', self.dumbbell_callback, 10
        )
        self.spawn_count = 0
        self.get_logger().info('Dumbbell Spawner (Harmonic/Template Mode) ready.')

    def dumbbell_callback(self, msg: String) -> None:
        try:
            color_name, weight_str = msg.data.split(',')
            weight = float(weight_str)
        except ValueError:
            self.get_logger().warn(f"Received invalid format: {msg.data}")
            return

        self.spawn_count += 1
        entity_name = f'dumbbell_{self.spawn_count}_{color_name}'
        
        # 4. Get RGBA string, default to white if unknown
        rgba_values = self.color_map.get(color_name, '1.0 1.0 1.0 1.0')

        # 5. Create the specific XML string for this dumbbell
        # Replace the placeholder in the file with the real color
        if self.template_content:
            robot_xml = self.template_content.replace('COLOR_RGBA', rgba_values)
        else:
            self.get_logger().error("Template content is empty!")
            return

        # Calculate position to avoid stacking
        x_pos = 0.5 * self.spawn_count
        
        self.get_logger().info(f'Spawning {color_name} dumbbell at x={x_pos}')

        # 6. Spawn using -string (passing the modified XML directly)
        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', 'sortabot_world',
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
