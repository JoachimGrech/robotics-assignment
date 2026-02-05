import rclpy
from rclpy.node import Node

class ChildABot(Node):
    def __init__(self):
        super().__init__('childabot')
        self.get_logger().info("ChildABot stub started")

def main(args=None):
    rclpy.init(args=args)
    node = ChildABot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()