#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from sortabot_actions.action import GoTo
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class SortabotActionServer(Node):
    def __init__(self):
        super().__init__('sortabot_action_server')
        self.get_logger().info('Action Server Starting (Real Controller)...')
        
        # Action Server
        self._action_server = ActionServer(
            self,
            GoTo,
            'go_to_location',
            self.execute_callback)

        # Publisher for velocities
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for position
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.last_odom_time = 0.0

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.last_odom_time = time.time()
        
        # Convert orientation to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def execute_callback(self, goal_handle):
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
