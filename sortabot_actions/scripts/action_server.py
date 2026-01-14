#!/usr/bin/env python3

import time
import math
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

# Assuming package name is sortabot_actions
from sortabot_actions.action import GoTo

class SortabotActionServer(Node):

    def __init__(self):
        super().__init__('sortabot_action_server')
        self.get_logger().info('Action Server Starting...')
        
        self._action_server = ActionServer(
            self,
            GoTo,
            'go_to_location',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        goal = goal_handle.request
        target_x = goal.x
        target_y = goal.y
        self.get_logger().info(f'Target: ({target_x}, {target_y})')

        feedback_msg = GoTo.Feedback()
        
        # Mock movement (simulated over 10 steps)
        # In connection with Ben's environment, this would call the Sim API
        steps = 10
        start_dist = 10.0 # Mock start distance
        
        for i in range(steps):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return GoTo.Result()

            feedback_msg.distance_to_goal = start_dist - (i * (start_dist / steps))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5) # Simulate time passing
            
        goal_handle.succeed()
        
        result = GoTo.Result()
        result.success = True
        result.message = f"Arrived at location ({target_x}, {target_y}) (Mock)"
        return result

def main(args=None):
    rclpy.init(args=args)
    node = SortabotActionServer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
