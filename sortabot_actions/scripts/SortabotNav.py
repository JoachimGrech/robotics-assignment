#!/usr/bin/env python3
"""
A* implementation used below is adapted from:
https://www.redblobgames.com/pathfinding/a-star/implementation.html
"""
import math
import heapq
import random
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class SortabotRLNode(Node):

    def __init__(self):
        super().__init__("sortabot_rl_node")


        # Publishers / Subscribers

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        self.control_timer = self.create_timer(0.2, self.control_loop)


        # Robot state

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.lidar_ranges = None
        self.lidar_angle_min = 0.0
        self.lidar_angle_inc = 0.0


        # Occupancy Grid

        self.resolution = 0.2  # meters per cell
        self.grid_size = 100   # 20m x 20m
        self.grid_origin = self.grid_size // 2

        self.occ_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.int8)


        # High-level memory

        self.marked_targets = {}  # label -> (gx, gy)
        self.current_path = []
        self.current_goal = None

        self.holding_object = None

 
        # RL scaffolding

        self.action_costs = {
             "ROAM":    -1.0,
             "MARK":    -0.5,
             "GOTO":    -2.0,
             "PICKUP":  -1.5,
             "PUTDOWN": -1.0
         }
        
        self.rewards = {
             "BIN_SUCCESS":  50.0,
             "WRONG_BIN":   -20.0,
        }

        self.state = "ROAM"

        self.get_logger().info("Sortabot RL node initialised")


    # Callbacks

    def lidar_callback(self, msg: LaserScan):
        self.lidar_ranges = msg.ranges
        self.lidar_angle_min = msg.angle_min
        self.lidar_angle_inc = msg.angle_increment

        self.update_occupancy_grid(msg)

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])


    # Occupancy grid

    def update_occupancy_grid(self, scan: LaserScan):
        for i, r in enumerate(scan.ranges):
            if math.isinf(r) or math.isnan(r):
                continue

            angle = scan.angle_min + i * scan.angle_increment
            wx = self.x + r * math.cos(self.yaw + angle)
            wy = self.y + r * math.sin(self.yaw + angle)

            gx = int(wx / self.resolution) + self.grid_origin
            gy = int(wy / self.resolution) + self.grid_origin

            if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                self.occ_grid[gx, gy] = 1


    # Core control loop

    def control_loop(self):
        if self.state == "ROAM":
            self.roam()

        elif self.state == "GOTO":
            self.follow_path()


    # LIDAR utilities

    def is_angle_clear(self, relative_angle, distance=1.0):
        if self.lidar_ranges is None:
            return False

        angle = relative_angle
        index = int((angle - self.lidar_angle_min) / self.lidar_angle_inc)

        if index < 0 or index >= len(self.lidar_ranges):
            return False

        return self.lidar_ranges[index] > distance


    # Motion primitives

    def rotate_in_place(self, angle):
        msg = Twist()
        msg.angular.z = 0.5 if angle > 0 else -0.5
        duration = abs(angle) / 0.5

        self.cmd_pub.publish(msg)
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=duration))

        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def move_forward(self, distance=1.0):
        msg = Twist()
        msg.linear.x = 0.3
        duration = distance / 0.3

        self.cmd_pub.publish(msg)
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=duration))

        msg.linear.x = 0.0
        self.cmd_pub.publish(msg)


    # Roaming logic

    def roam(self):
        candidate_sets = [
            lambda: random.uniform(-math.pi/4, math.pi/4),
            lambda: random.choice([
                random.uniform(math.radians(70), math.radians(80)),
                random.uniform(-math.radians(80), -math.radians(70))
            ]),
            lambda: random.choice([math.pi/2, -math.pi/2]),
            lambda: math.pi
        ]

        for pick_angle in candidate_sets:
            angle = pick_angle()

            if self.is_angle_clear(angle):
                self.rotate_in_place(angle)
                self.move_forward(1.0)
                return

        self.get_logger().warn("Roam blocked in all directions")


    # MARK action (needs labels from vision)

    def mark(self, label, world_x, world_y):
        gx = int(world_x / self.resolution) + self.grid_origin
        gy = int(world_y / self.resolution) + self.grid_origin
        self.marked_targets[label] = (gx, gy)


    # GOTO using A*

    def goto(self, label):
        if label not in self.marked_targets:
            return

        start = (
            int(self.x / self.resolution) + self.grid_origin,
            int(self.y / self.resolution) + self.grid_origin
        )

        goal = self.marked_targets[label]
        self.current_path = self.astar(start, goal)
        self.state = "GOTO"

    def follow_path(self):
        if not self.current_path:
            self.state = "ROAM"
            return

        gx, gy = self.current_path[0]

        wx = (gx - self.grid_origin) * self.resolution
        wy = (gy - self.grid_origin) * self.resolution

        dx = wx - self.x
        dy = wy - self.y

        target_yaw = math.atan2(dy, dx)
        turn = self.normalize_angle(target_yaw - self.yaw)

        if abs(turn) > 0.1:
            self.rotate_in_place(turn)
            return

        if not self.is_angle_clear(0.0):
            self.roam()
            return

        self.move_forward(1.0)
        self.current_path.pop(0)

 
    # A* implementation
    # Source adapted from: https://www.redblobgames.com/pathfinding/a-star/


    def astar(self, start, goal):
        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        neighbors = [(1,0), (-1,0), (0,1), (0,-1)]

        open_set = []
        heapq.heappush(open_set, (0, start))

        came_from = {}
        g_score = {start: 0}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for dx, dy in neighbors:
                nxt = (current[0] + dx, current[1] + dy)

                if not (0 <= nxt[0] < self.grid_size and 0 <= nxt[1] < self.grid_size):
                    continue
                if self.occ_grid[nxt[0], nxt[1]] == 1:
                    continue

                tentative = g_score[current] + 1
                if nxt not in g_score or tentative < g_score[nxt]:
                    came_from[nxt] = current
                    g_score[nxt] = tentative
                    f = tentative + heuristic(nxt, goal)
                    heapq.heappush(open_set, (f, nxt))

        return []

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path


    # Manipulation (stubs)


    def pick_up(self, hand="left"):
        # Would command left_joint1 / right_joint1
        self.holding_object = {"hand": hand}

    def put_down(self):
        # Reward logic would live here
        # if correct bin: reward += BIN_SUCCESS
        self.holding_object = None



    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a


def main():
    rclpy.init()
    node = SortabotRLNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()