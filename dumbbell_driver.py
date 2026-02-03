import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class SmartDumbbell(Node):
    def __init__(self):
        super().__init__('dumbbell_driver')
        
        self.available_colors = ['red', 'blue', 'green', 'yellow']
        
        self.MAX_WEIGHT_LIMIT = 100.0 
        self.total_weight_spawned = 0.0
        
        self.publisher_ = self.create_publisher(String, 'dumbbell_info', 10)
        
        self.create_timer(1.0, self.broadcast_status)

        print(f"Max Capacity: {self.MAX_WEIGHT_LIMIT} kg")
        self.spawn_dumbbell()

    def spawn_dumbbell(self):
        if self.total_weight_spawned >= self.MAX_WEIGHT_LIMIT:
            print("Weight limit reached")
            print(f"Total Spawned: {self.total_weight_spawned:.2f} / {self.MAX_WEIGHT_LIMIT} kg")
            return

        new_color = random.choice(self.available_colors)
        new_weight = random.randint(5, 18)

        self.color = new_color
        self.weight = new_weight
        self.total_weight_spawned += new_weight

        print(f"New Dumbbell Spawned:")
        print(f"Colour: {self.color.upper()}")
        print(f"Weight: {self.weight} kg")
        print(f"Total: {self.total_weight_spawned:.2f} kg so far")

        next_interval = random.randint(15, 60)
        print(f"Next item arriving in {next_interval} seconds")
        
        self.spawn_timer = self.create_timer(next_interval, self.timer_callback)

    def timer_callback(self):
        self.spawn_timer.cancel()
        self.spawn_dumbbell()

    def broadcast_status(self):
        if self.total_weight_spawned == 0.0:
            return
            
        msg = String()
        msg.data = f"{self.color},{self.weight}"
        self.publisher_.publish(msg)

def main(args=None):
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