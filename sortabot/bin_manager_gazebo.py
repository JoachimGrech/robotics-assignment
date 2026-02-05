import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SmartBin(Node):
    def __init__(self):
        super().__init__('smart_bin_manager')
        
        try:
            self.declare_parameter('use_sim_time', False)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass
            
        self.bin_data = {
            'red': 0.0,
            'blue': 0.0,
            'green': 0.0,
            'yellow': 0.0
        }

        # 1. Listen for status checks (Optional)
        self.ping_subscription = self.create_subscription(
            String,
            'ping_bin',
            self.ping_callback,
            10)

        # 2. Listen for items! 
        # CHANGED TOPIC: 'deposit_item' -> 'dumbbell_info'
        # This matches what your driver publishes.
        self.deposit_subscription = self.create_subscription(
            String,
            'dumbbell_info',
            self.deposit_callback,
            10)
            
        self.get_logger().info("Smart Bin Manager Initialized.")
        self.get_logger().info("Listening for items on '/dumbbell_info':")

    def ping_callback(self, msg):
        requested_color = msg.data.lower()
        
        if requested_color in self.bin_data:
            current_weight = self.bin_data[requested_color]
            self.get_logger().info(f"Bin {requested_color.upper()} reports: {current_weight} kg.")
        else:
            self.get_logger().error(f"ERROR: Bin color '{requested_color}' does not exist.")

    def deposit_callback(self, msg):
        try:
            # Expected format: "color,weight" (e.g., "red,10")
            data = msg.data.split(',')
            color = data[0].lower()
            weight = float(data[1])

            if color in self.bin_data:
                # Add the weight
                self.bin_data[color] += weight
                new_total = self.bin_data[color]
                self.get_logger().info(f"Received {weight} kg for {color.upper()} bin.")
                self.get_logger().info(f"Total {color.upper()} stored: {new_total} kg")
            else:
                self.get_logger().warn(f"Ignored unknown item: {color}")
                
        except ValueError:
            self.get_logger().error("ERROR: Invalid weight format.")
        except IndexError:
            self.get_logger().error("ERROR: Invalid message format. Use 'color,weight'")

def main(args=None):
    rclpy.init(args=args)
    node = SmartBin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
