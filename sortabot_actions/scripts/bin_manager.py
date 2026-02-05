import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SmartBin(Node):
    def __init__(self):
        super().__init__('smart_bin_manager')
        
        self.bin_data = {
            'red': 0.0,
            'blue': 0.0,
            'green': 0.0,
            'yellow': 0.0
        }

        self.ping_subscription = self.create_subscription(
            String,
            'ping_bin',
            self.ping_callback,
            10)

        self.deposit_subscription = self.create_subscription(
            String,
            'deposit_item',
            self.deposit_callback,
            10)
            
        self.get_logger().info("Smart Bin Manager Initialized.")
        self.get_logger().info("Listening for robot deposits on '/deposit_item'")
        self.get_logger().info("Listening for status checks on '/ping_bin'")

    def ping_callback(self, msg):
        requested_color = msg.data.lower()
        
        if requested_color in self.bin_data:
            current_weight = self.bin_data[requested_color]
            self.get_logger().info(f"Bin {requested_color.upper()} reports: {current_weight} kg.")
        else:
            self.get_logger().error(f"ERROR: Bin color '{requested_color}' does not exist.")

    def deposit_callback(self, msg):
        try:
            data = msg.data.split(',')
            color = data[0].lower()
            weight = float(data[1])

            if color in self.bin_data:
                # Add the weight
                self.bin_data[color] += weight
                
                new_total = self.bin_data[color]
                self.get_logger().info(f"Robot deposited {weight} kg into {color.upper()} bin.")
                self.get_logger().info(f"New Total for {color.upper()}: {new_total} kg")
            else:
                self.get_logger().error(f"ERROR: Robot tried to put item in unknown bin: {color}")
                
        except:
            self.get_logger().error("ERROR: Invalid message format. Use 'color,weight'")

def main(args=None):
    rclpy.init(args=args)
    node = SmartBin()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
