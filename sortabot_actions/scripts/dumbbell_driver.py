import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class SmartDumbbell(Node):
    def __init__(self) -> None:
        super().__init__('dumbbell_driver')

        # Colors must match the keys in the spawner's color_map
        self.available_colors = ['red', 'blue', 'green', 'yellow']

        self.max_weight_limit = 100.0
        self.total_weight_spawned = 0.0

        # Publisher: sends "color,weight"
        self.publisher_ = self.create_publisher(String, 'dumbbell_info', 10)

        self.get_logger().info(f'Dumbbell Driver Started. Max Capacity: {self.max_weight_limit} kg')
        
        # Spawn the first one immediately
        self.spawn_dumbbell()

    def spawn_dumbbell(self) -> None:
        """Create a new dumbbell, publish msg, schedule next."""
        if self.total_weight_spawned >= self.max_weight_limit:
            self.get_logger().info('Max weight limit reached. Stopping generation.')
            return

        # Randomly choose attributes
        new_color = random.choice(self.available_colors)
        new_weight = random.randint(5, 15)  # Random weight between 5kg and 15kg

        self.total_weight_spawned += new_weight

        # Log info
        self.get_logger().info(
            f'Generating {new_color.upper()} dumbbell ({new_weight}kg). Total: {self.total_weight_spawned}kg'
        )

        # Publish the command to the spawner
        msg = String()
        msg.data = f'{new_color},{new_weight}'
        self.publisher_.publish(msg)

        # Schedule the next spawn (e.g., between 5 and 10 seconds)
        next_interval = random.randint(5, 10)
        self.spawn_timer = self.create_timer(float(next_interval), self.timer_callback)

    def timer_callback(self) -> None:
        """One-shot timer fired."""
        self.spawn_timer.cancel()
        self.spawn_dumbbell()

def main(args=None) -> None:
    rclpy.init(args=args)
    node = SmartDumbbell()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
