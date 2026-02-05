import os
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory
import xacro

class WorldManager(Node):
    def __init__(self):
        super().__init__('world_manager')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        
        self.get_logger().info("Connecting to Simulation...")
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')

        self.spawn_objects()
        self.spawn_robot()

    def spawn_objects(self):
        # 1. Spawn the Bins
        pkg_share = get_package_share_directory('sortabot')
        bins_path = os.path.join(pkg_share, 'urdf', 'bins.urdf')
        
        with open(bins_path, 'r') as f:
            bins_xml = f.read()

        self.spawn_entity("sorting_bins", bins_xml, 3.0, 0.0, 0.0)

    def spawn_robot(self):
        # 2. Spawn Sortabot (Convert Xacro to XML first)
        pkg_share = get_package_share_directory('sortabot')
        xacro_file = os.path.join(pkg_share, 'urdf', 'sortabot.urdf.xacro')
        
        # Process the Xacro file
        doc = xacro.process_file(xacro_file)
        robot_xml = doc.toxml()

        # Spawn at (0,0)
        self.spawn_entity("sortabot", robot_xml, 0.0, 0.0, 0.2)

    def spawn_entity(self, name, xml_content, x, y, z):
        req = SpawnEntity.Request()
        req.name = name
        req.xml = xml_content
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = float(z)
        
        self.spawn_client.call_async(req)
        self.get_logger().info(f"Spawned: {name}")

def main(args=None):
    rclpy.init(args=args)
    node = WorldManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
