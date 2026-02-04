#!/usr/bin/env python3
"""
lidar_navigator_with_goto_astar.py

ROS 2 node for Sortabot
- Roams using ±45° avoidance logic
- Maintains occupancy grid updated from LIDAR
- GoTo method: uses A* to navigate to target, reorients if blocked

A* implementation used below is adapted from:
https://www.redblobgames.com/pathfinding/a-star/implementation.html
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math
import random
import heapq
import cv2
import yaml


# Simple A* implementation (from redblobgames)

def heuristic(a, b):
    # Manhattan distance
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_search(grid, start, goal):
    """
    grid: 2D numpy array (0=free, 1=obstacle)
    start, goal: tuples (x, y)
    returns: list of tuples for path (or None)
    Source: https://www.redblobgames.com/pathfinding/a-star/implementation.html
    """
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        x, y = current
        neighbors = [(x+1,y),(x-1,y),(x,y+1),(x,y-1)]  # 4-connectivity
        for nx, ny in neighbors:
            if not (0 <= nx < grid.shape[0] and 0 <= ny < grid.shape[1]):
                continue
            if grid[nx, ny] == 1:  # obstacle
                continue
            neighbor = (nx, ny)
            tentative_g = g_score[current] + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f = tentative_g + heuristic(goal, neighbor)
                heapq.heappush(open_set, (f, neighbor))
    return None  # no path found


# ROS 2 Node

class LidarNavigator(Node):
    def __init__(self):
        super().__init__('lidar_navigator_with_goto_astar')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10
        )

        # Movement parameters
        self.forward_distance = 1.0  # meters per step
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.ranges = []
        self.ranges_min = -math.pi
        self.ranges_max = math.pi
        self.angle_increment = 0.0

        # Occupancy grid parameters
        self.grid_resolution = 0.05
        self.grid_size = (200, 200)
        self.occupancy_grid = -1 * np.ones(self.grid_size, dtype=np.int8)
        self.origin = [self.grid_size[0]//2, self.grid_size[1]//2]  # robot starts center

        self.get_logger().info("LidarNavigator with GoTo + simple A* started.")


    # LIDAR callback & occupancy grid update

    def laser_callback(self, msg: LaserScan):
        self.ranges = msg.ranges
        self.ranges_min = msg.angle_min
        self.ranges_max = msg.angle_max
        self.angle_increment = msg.angle_increment

        self.update_occupancy_grid()
        # For normal roaming uncomment:
        # self.navigate_step()

    def update_occupancy_grid(self):
        if not self.ranges:
            return
        for i,distance in enumerate(self.ranges):
            if distance == float('inf') or distance <= 0.0:
                continue
            angle = self.ranges_min + i*self.angle_increment
            x = distance*math.cos(angle)
            y = distance*math.sin(angle)
            gx = int(self.origin[0] + x/self.grid_resolution)
            gy = int(self.origin[1] + y/self.grid_resolution)
            if 0 <= gx < self.grid_size[0] and 0 <= gy < self.grid_size[1]:
                self.occupancy_grid[gx,gy] = 100  # occupied

    def save_occupancy_grid(self, filename_prefix="map"):
        img = np.zeros_like(self.occupancy_grid,dtype=np.uint8)
        img[self.occupancy_grid==100]=0
        img[self.occupancy_grid==0]=255
        img[self.occupancy_grid==-1]=127
        cv2.imwrite(f"{filename_prefix}.pgm",img)
        meta={"image":f"{filename_prefix}.pgm","resolution":self.grid_resolution,
              "origin":[0,0,0],"negate":0,"occupied_thresh":0.65,"free_thresh":0.196}
        with open(f"{filename_prefix}.yaml","w") as f:
            yaml.dump(meta,f)


    # Roaming / avoidance

    def navigate_step(self):
        if not self.ranges:
            return
        candidate_angles = [random.uniform(-45,45)] + [70,-70,80,-80,90,-90,180,-180]
        for angle in candidate_angles:
            if self.is_path_clear(angle):
                self.execute_move(angle)
                return
        self.stop_robot()

    def is_path_clear(self, angle_deg):
        if not self.ranges:
            return False
        scan_angle = math.radians(angle_deg)
        index = int((scan_angle - self.ranges_min)/self.angle_increment)
        index = max(0,min(len(self.ranges)-1,index))
        distance = self.ranges[index]
        return distance > self.forward_distance

    def execute_move(self, angle_deg):
        self.rotate_relative(angle_deg)
        self.move_forward(self.forward_distance)

    def rotate_relative(self, angle_deg):
        twist = Twist()
        target_rad = math.radians(angle_deg)
        direction = 1.0 if target_rad >= 0 else -1.0
        duration = abs(target_rad)/self.angular_speed
        twist.angular.z = direction*self.angular_speed
        start_time = self.get_clock().now().nanoseconds/1e9
        while (self.get_clock().now().nanoseconds/1e9 - start_time) < duration:
            self.publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)
        self.stop_robot()

    def move_forward(self, distance):
        twist = Twist()
        duration = distance/self.linear_speed
        twist.linear.x = self.linear_speed
        start_time = self.get_clock().now().nanoseconds/1e9
        while (self.get_clock().now().nanoseconds/1e9 - start_time) < duration:
            self.publisher.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.01)
        self.stop_robot()

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)


    # GoTo using A*

    def go_to(self, target_x, target_y, radius=0.2):
        """Navigate to target (meters) using occupancy grid + A*"""
        while True:
            start = (self.origin[0], self.origin[1])
            goal = (int(self.origin[0]+target_x/self.grid_resolution),
                    int(self.origin[1]+target_y/self.grid_resolution))
            # convert occupancy grid to 0=free,1=obstacle for A*
            grid_binary = np.where(self.occupancy_grid==100,1,0)
            path = a_star_search(grid_binary, start, goal)
            if path is None:
                self.get_logger().info("No path found, reorienting with roaming...")
                self.navigate_step()
                continue

            for gx,gy in path:
                rel_x = (gx - self.origin[0])*self.grid_resolution
                rel_y = (gy - self.origin[1])*self.grid_resolution
                angle_to_goal = math.degrees(math.atan2(rel_y, rel_x))
                if not self.is_path_clear(angle_to_goal):
                    self.get_logger().info("Obstacle detected, reorienting...")
                    self.navigate_step()
                    break
                else:
                    self.execute_move(angle_to_goal)
                if math.hypot(rel_x,rel_y) <= radius:
                    return
            else:
                return


def main(args=None):
    rclpy.init(args=args)
    node = LidarNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_occupancy_grid()
        node.get_logger().info("Occupancy grid saved.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
