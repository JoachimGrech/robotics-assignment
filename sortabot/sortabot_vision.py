import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class SortabotVision(Node):
    def __init__(self):
        super().__init__('vision_processor')
        
        # 1. Initialize CV Bridge
        self.bridge = CvBridge()
        
        # 2. Subscribe to Camera
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # 3. Define Color Ranges (in HSV)
        # These ranges might need tuning based on the Gazebo lighting and object colors
        self.color_ranges = {
            'red':    ([0, 100, 100], [10, 255, 255]),
            'green':  ([40, 100, 100], [80, 255, 255]),
            'blue':   ([100, 100, 100], [140, 255, 255]),
            'yellow': ([20, 100, 100], [30, 255, 255])
        }
        
        self.get_logger().info("Sortabot Vision Node Started. Drawing bounding boxes...")

    def image_callback(self, msg):
        # 1. Convert ROS Image to OpenCV Image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Could not convert image: {e}")
            return

        # 2. Process Image
        processed_image = self.detect_objects(cv_image)

        # 3. Display Image (Requires GUI support)
        # Note: In some headless environments this might fail or need a different approach
        cv2.imshow("Sortabot Vision", processed_image)
        cv2.waitKey(1)

    def detect_objects(self, image):
        # Convert to HSV for better color segmentation
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        for color_name, (lower, upper) in self.color_ranges.items():
            lower_np = np.array(lower, dtype="uint8")
            upper_np = np.array(upper, dtype="uint8")
            
            # Create Mask
            mask = cv2.inRange(hsv_image, lower_np, upper_np)
            
            # Find Contours
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Filter small noise (adjust threshold as needed)
                if area > 500:
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Draw Bounding Box
                    color_bgr = self.get_color_bgr(color_name)
                    cv2.rectangle(image, (x, y), (x + w, y + h), color_bgr, 2)
                    cv2.putText(image, f"{color_name}", (x, y - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)
                    
        return image

    def get_color_bgr(self, name):
        if name == 'red': return (0, 0, 255)
        if name == 'green': return (0, 255, 0)
        if name == 'blue': return (255, 0, 0)
        if name == 'yellow': return (0, 255, 255)
        return (255, 255, 255)

def main(args=None):
    rclpy.init(args=args)
    node = SortabotVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
