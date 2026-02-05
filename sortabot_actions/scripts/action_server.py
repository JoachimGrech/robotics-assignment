#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from sortabot_actions.action import GoTo, GraspObject, WeighObject, DepositObject
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import time

class SortabotActionServer(Node):
    def __init__(self):
        super().__init__('sortabot_action_server')
        self.get_logger().info('Action Server Starting (Real Controller)...')
        
        # Action Server
        self._goto_server = ActionServer(
            self,
            GoTo,
            'go_to_location',
            self.execute_goto_callback)

        self._grasp_server = ActionServer(
            self,
            GraspObject,
            'grasp_object',
            self.execute_grasp_callback)

        self._weigh_server = ActionServer(
            self,
            WeighObject,
            'weigh_object',
            self.execute_weigh_callback)

        self._deposit_server = ActionServer(
            self,
            DepositObject,
            'deposit_object',
            self.execute_deposit_callback)

        # Publisher for velocities
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for position
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.last_odom_time = 0.0

        # State for grasping and weighing
        self.grasped_objects = {
            'left': None,  # Stores { 'id': str, 'weight': float, 'color': str }
            'right': None
        }
        self.known_objects = {} # object_id -> { weight, color }

        # Subscriber for dumbbell info (to know weights)
        self.dumbbell_info_sub = self.create_subscription(
            String,
            'dumbbell_info',
            self.dumbbell_info_callback,
            10)

        # Publisher for depositing
        self.deposit_pub = self.create_publisher(String, 'deposit_item', 10)

    def dumbbell_info_callback(self, msg):
        # Format: "color,weight" - we'll simulate IDs based on order or color
        try:
            color, weight = msg.data.split(',')
            obj_id = f"{color}_{len(self.known_objects)}"
            self.known_objects[obj_id] = {
                'color': color,
                'weight': float(weight)
            }
            self.get_logger().info(f"Registered new object: {obj_id} ({weight}kg)")
        except Exception as e:
            self.get_logger().error(f"Failed to parse dumbbell info: {e}")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.last_odom_time = time.time()
        
        # Convert orientation to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def execute_goto_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        target_x = goal_handle.request.x
        target_y = goal_handle.request.y
        
        feedback_msg = GoTo.Feedback()
        twist = Twist()
        
        rate = self.create_rate(10) # 10Hz
        
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.stop_robot()
                return GoTo.Result()

            # Check for stale odom
            if time.time() - self.last_odom_time > 2.0:
                self.get_logger().warning('Odom is stale! Check topic bridge.')

            # Calculate error
            dx = target_x - self.current_x
            dy = target_y - self.current_y
            distance = math.sqrt(dx**2 + dy**2)
            
            # DEBUG LOGGING
            self.get_logger().info(f'Current: ({self.current_x:.2f}, {self.current_y:.2f}) | Dist: {distance:.2f}', throttle_duration_sec=1.0)
            
            feedback_msg.distance_to_goal = distance
            goal_handle.publish_feedback(feedback_msg)

            # Check if arrived
            if distance < 0.15:
                break

            # Controller
            target_angle = math.atan2(dy, dx)
            angle_error = target_angle - self.current_yaw
            while angle_error > math.pi: angle_error -= 2*math.pi
            while angle_error < -math.pi: angle_error += 2*math.pi

            twist = Twist()
            if abs(angle_error) > 0.4:
                # Rotate first
                twist.angular.z = 0.8 if angle_error > 0 else -0.8
                twist.linear.x = 0.0
            else:
                # Drive and adjust
                twist.linear.x = 0.3
                twist.angular.z = angle_error * 1.5
            
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        self.stop_robot()
        goal_handle.succeed()
        
        result = GoTo.Result()
        result.success = True
        result.message = f"Arrived at ({target_x:.2f}, {target_y:.2f})"
        return result

    def execute_grasp_callback(self, goal_handle):
        self.get_logger().info('Executing Grasp...')
        arm = goal_handle.request.arm_side.lower()
        obj_id = goal_handle.request.object_id
        
        result = GraspObject.Result()
        
        if arm not in ['left', 'right']:
            result.success = False
            result.message = f"Invalid arm: {arm}"
            goal_handle.abort()
            return result

        if obj_id not in self.known_objects:
            result.success = False
            result.message = f"Object {obj_id} not found."
            goal_handle.abort()
            return result

        if self.grasped_objects[arm] is not None:
            result.success = False
            result.message = f"{arm.capitalize()} arm is already holding something."
            goal_handle.abort()
            return result

        # Simulate grasping
        self.grasped_objects[arm] = self.known_objects[obj_id]
        self.grasped_objects[arm]['id'] = obj_id
        
        self.get_logger().info(f"Successfully grasped {obj_id} with {arm} arm.")
        goal_handle.succeed()
        result.success = True
        result.message = f"Grasped {obj_id}"
        return result

    def execute_weigh_callback(self, goal_handle):
        self.get_logger().info('Executing Weighing...')
        arm = goal_handle.request.arm_side.lower()
        
        result = WeighObject.Result()
        
        if arm not in ['left', 'right'] or self.grasped_objects[arm] is None:
            result.success = False
            goal_handle.abort()
            return result

        # Simulate weighing (returning the known weight)
        weight = self.grasped_objects[arm]['weight']
        
        # Feedback simulation
        feedback = WeighObject.Feedback()
        for i in range(5):
            feedback.current_reading = weight + (0.5 - math.sin(i)) # simulate fluctuation
            goal_handle.publish_feedback(feedback)
            time.sleep(0.2)

        result.weight = weight
        result.success = True
        goal_handle.succeed()
        return result

    def execute_deposit_callback(self, goal_handle):
        self.get_logger().info('Executing Deposit...')
        arm = goal_handle.request.arm_side.lower()
        bin_color = goal_handle.request.bin_color.lower()
        
        result = DepositObject.Result()
        
        if arm not in ['left', 'right'] or self.grasped_objects[arm] is None:
            result.success = False
            goal_handle.abort()
            return result

        obj = self.grasped_objects[arm]
        
        # In this simulation, we check if the bin color matches our goal or just deposit
        # The bin_manager expects "color,weight"
        deposit_msg = String()
        deposit_msg.data = f"{bin_color},{obj['weight']}"
        self.deposit_pub.publish(deposit_msg)

        self.get_logger().info(f"Deposited {obj['id']} into {bin_color} bin.")
        
        # Clear arm state
        self.grasped_objects[arm] = None
        
        result.success = True
        goal_handle.succeed()
        return result

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SortabotActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
