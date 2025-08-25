#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Twist
import numpy as np

class DwaPlannerNode(Node):
    def __init__(self):
        super().__init__('dwa_planner_node')

        # Subscriptions
        self.recovery_sub = self.create_subscription(
            Float32MultiArray,
            '/dead_end_detection/recovery_points',
            self.recovery_callback,
            10
        )
        self.costmap_sub = self.create_subscription(
            MarkerArray,
            '/cost_layer',
            self.costmap_callback,
            10
        )

        # Publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Internal state
        self.recovery_points = []  # List of (x, y) tuples
        self.costmap = {}          # (x, y) -> cost/prob

        # Robot parameters (tune for your robot)
        self.max_speed = 0.5
        self.max_omega = 1.0
        self.dt = 0.1

        # Timer for planning loop
        self.create_timer(0.1, self.plan_and_publish)

        self.get_logger().info('DWA Planner Node initialized')

    def recovery_callback(self, msg):
        # Parse recovery points from Float32MultiArray
        # Example: [x1, y1, x2, y2, ...]
        data = np.array(msg.data)
        if len(data) % 2 == 0:
            self.recovery_points = [(data[i], data[i+1]) for i in range(0, len(data), 2)]
        else:
            self.get_logger().warn('Recovery points array has odd length!')

    def costmap_callback(self, msg):
        # Parse costmap from MarkerArray
        self.costmap = {}
        for marker in msg.markers:
            x = marker.pose.position.x
            y = marker.pose.position.y
            cost = marker.color.r  # Assuming red channel encodes cost/prob
            self.costmap[(x, y)] = cost

    def plan_and_publish(self):
        if not self.recovery_points or not self.costmap:
            return  # Wait for data

        # Get current robot pose (for demo, assume at (0,0,0))
        robot_x, robot_y, robot_theta = 0.0, 0.0, 0.0

        # DWA: Sample velocities
        best_score = -float('inf')
        best_v = 0.0
        best_omega = 0.0

        for v in np.linspace(0, self.max_speed, 5):
            for omega in np.linspace(-self.max_omega, self.max_omega, 9):
                # Simulate forward
                x, y, theta = robot_x, robot_y, robot_theta
                for _ in range(10):  # Simulate 1 second ahead
                    x += v * np.cos(theta) * self.dt
                    y += v * np.sin(theta) * self.dt
                    theta += omega * self.dt

                # Score: prefer low cost, close to recovery point
                cost = self.get_cost(x, y)
                goal_score = -min(np.hypot(x - rx, y - ry) for rx, ry in self.recovery_points)
                total_score = -cost * 10.0 + goal_score * 2.0 - abs(omega) * 0.1

                if total_score > best_score:
                    best_score = total_score
                    best_v = v
                    best_omega = omega

        # Publish best command
        cmd = Twist()
        cmd.linear.x = best_v
        cmd.angular.z = best_omega
        self.cmd_pub.publish(cmd)

    def get_cost(self, x, y):
        # Find nearest costmap cell
        min_dist = float('inf')
        min_cost = 0.0
        for (cx, cy), cost in self.costmap.items():
            dist = np.hypot(x - cx, y - cy)
            if dist < min_dist:
                min_dist = dist
                min_cost = cost
        return min_cost

def main(args=None):
    rclpy.init(args=args)
    node = DwaPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()