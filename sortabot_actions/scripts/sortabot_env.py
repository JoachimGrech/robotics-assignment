import gym
from gym import spaces
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sortabot_actions.action import GoTo

class SortabotEnv(gym.Env):
    """
    Custom Environment that follows gym interface.
    This environment interfaces with the ROS 2 Action Server to control the robot.
    """
    metadata = {'render.modes': ['human']}

    def __init__(self, ros_node: Node):
        super(SortabotEnv, self).__init__()
        self.node = ros_node
        
        # Define Action Space:
        # 0: Move to Bin A
        # 1: Move to Bin B
        # 2: Move to Collection Zone
        # 3: Pick Object
        # 4: Place Object
        self.action_space = spaces.Discrete(5)

        # Define Observation Space (Simplified):
        # [robot_x, robot_y, holding_object (0/1)]
        low = np.array([-10.0, -10.0, 0])
        high = np.array([10.0, 10.0, 1])
        self.observation_space = spaces.Box(low, high, dtype=np.float32)

        # ROS 2 Action Client
        self._action_client = ActionClient(self.node, GoTo, 'go_to_location')
        self.node.get_logger().info('Waiting for action server...')
        # Note: In a real training loop, we might wait here, but for now we skip blocking
        # self._action_client.wait_for_server()

        self.state = np.array([0.0, 0.0, 0.0])

    def step(self, action):
        """
        Execute one time step within the environment
        """
        self.node.get_logger().info(f'Step Action: {action}')
        
        # Map action to ROS 2 Goal
        # Example: Simple logic mapping discrete actions to coordinates
        if action == 0: # Bin A
            self._send_goal(2.0, 2.0)
        elif action == 1: # Bin B
            self._send_goal(-2.0, 2.0)
        elif action == 2: # Collection
            self._send_goal(0.0, 0.0)
        
        # In a real step, we would wait for the action to complete or return immediate feedback
        # For now, we return a mock next_state and reward
        
        reward = 1.0 # Placeholder
        done = False
        info = {}
        
        return self.state, reward, done, info

    def reset(self):
        """
        Reset the state of the environment to an initial state
        """
        self.state = np.array([0.0, 0.0, 0.0])
        return self.state

    def _send_goal(self, x, y):
        goal_msg = GoTo.Goal()
        goal_msg.x = float(x)
        goal_msg.y = float(y)
        goal_msg.theta = 0.0
        
        self.node.get_logger().info(f'Sending goal: ({x}, {y})')
        self._action_client.send_goal_async(goal_msg)

    def render(self, mode='human', close=False):
        pass
