#!/usr/bin/env python3

import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

# Note: This import will fail until the package is built and sourced
# from sortabot_actions.action import GoTo

class SortabotActionServer(Node):

    def __init__(self):
        super().__init__('sortabot_action_server')
        self.get_logger().info('Action Server Starting...')
        
        # Placeholder for the action server
        # self._action_server = ActionServer(
        #     self,
        #     GoTo,
        #     'go_to_location',
        #     self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        # Mock execution logic
        feedback_msg = GoTo.Feedback()
        
        for i in range(10):
            feedback_msg.distance_to_goal = 10.0 - (i * 1.0)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)
            
        goal_handle.succeed()
        
        result = GoTo.Result()
        result.success = True
        result.message = "Arrived at location (Mock)"
        return result

def main(args=None):
    rclpy.init(args=args)
    node = SortabotActionServer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
