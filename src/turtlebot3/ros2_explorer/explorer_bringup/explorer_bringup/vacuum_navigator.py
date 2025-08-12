#!/usr/bin/env python3
"""
vacuum_navigator.py

A simple ROS2 Python node for a vacuum robot that uses SLAM (e.g., Cartographer)
for localization, but does its own reactive navigation (no Nav2).

Features:
- Subscribes to /scan (sensor_msgs/LaserScan) to detect obstacles
- Uses TF (tf2) to read robot pose (map->base_link) when needed
- Publishes /cmd_vel (geometry_msgs/Twist) to drive the robot
- Simple state machine: FORWARD, TURN, SPIRAL
- Chooses turn direction based on which side has more free space
- Implements a spiral/wiggle behavior to explore area

Usage:
1. Run Cartographer (or any SLAM) to provide map and TF:
   ros2 launch cartographer_ros cartographer.launch.py
2. Run this node (make executable):
   chmod +x vacuum_navigator.py
   ros2 run <your_package> vacuum_navigator.py
   OR directly: ./vacuum_navigator.py

Adjust parameters either by editing or using ROS2 parameters when launching.
"""

import math
import random
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import tf2_ros
from tf2_ros import TransformException
from rclpy.duration import Duration


class State(Enum):
    FORWARD = 1
    TURN = 2
    SPIRAL = 3


class VacuumNavigator(Node):
    def __init__(self):
        super().__init__('vacuum_navigator')

        # Parameters (can be adjusted)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('forward_speed', 0.15)  # m/s
        self.declare_parameter('turn_speed', 0.6)      # rad/s
        self.declare_parameter('min_dist', 0.35)       # m (front obstacle threshold)
        self.declare_parameter('side_dist', 0.4)       # m (to consider side free)
        self.declare_parameter('spiral_forward', 0.08) # m/s when spiraling
        self.declare_parameter('spiral_turn_rate', 0.2)# rad/s base for spiral
        self.declare_parameter('front_angle', 15.0)    # degrees to check front
        self.declare_parameter('turn_duration_min', 0.6)
        self.declare_parameter('turn_duration_max', 1.6)
        self.declare_parameter('random_move_chance', 0.02)  # chance per cycle to change

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.turn_speed = self.get_parameter('turn_speed').get_parameter_value().double_value
        self.min_dist = self.get_parameter('min_dist').get_parameter_value().double_value
        self.side_dist = self.get_parameter('side_dist').get_parameter_value().double_value
        self.spiral_forward = self.get_parameter('spiral_forward').get_parameter_value().double_value
        self.spiral_turn_rate = self.get_parameter('spiral_turn_rate').get_parameter_value().double_value
        self.front_angle = self.get_parameter('front_angle').get_parameter_value().double_value
        self.turn_duration_min = self.get_parameter('turn_duration_min').get_parameter_value().double_value
        self.turn_duration_max = self.get_parameter('turn_duration_max').get_parameter_value().double_value
        self.random_move_chance = self.get_parameter('random_move_chance').get_parameter_value().double_value

        # Publisher and subscriber
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State
        self.state = State.FORWARD
        self.last_scan_time = self.get_clock().now()
        self.front_clear = True
        self.left_free = True
        self.right_free = True
        self.turn_end_time = None
        self.turn_direction = 1  # 1: left (CCW), -1: right (CW)

        # Timer to run control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.get_logger().info('VacuumNavigator started')

    def scan_callback(self, msg: LaserScan):
        # Process scan to find min distance in sectors
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        n = len(ranges)

        # Helper to get index for angle in radians
        def idx_for_angle(deg):
            rad = math.radians(deg)
            idx = int((rad - angle_min) / angle_increment)
            return max(0, min(n - 1, idx))

        # Front sector: -front_angle .. +front_angle
        front_left_idx = idx_for_angle(-self.front_angle)
        front_right_idx = idx_for_angle(self.front_angle)
        if front_left_idx <= front_right_idx:
            front_ranges = ranges[front_left_idx:front_right_idx + 1]
        else:
            # wrap-around case
            front_ranges = ranges[front_left_idx:] + ranges[:front_right_idx + 1]

        # Left sector: +30 deg .. +90 deg
        left_idx_start = idx_for_angle(30)
        left_idx_end = idx_for_angle(90)
        left_ranges = ranges[left_idx_start:left_idx_end + 1] if left_idx_start <= left_idx_end else ranges[left_idx_start:] + ranges[:left_idx_end + 1]

        # Right sector: -90 deg .. -30 deg
        right_idx_start = idx_for_angle(-90)
        right_idx_end = idx_for_angle(-30)
        right_ranges = ranges[right_idx_start:right_idx_end + 1] if right_idx_start <= right_idx_end else ranges[right_idx_start:] + ranges[:right_idx_end + 1]

        # Compute safe min with NaN/Inf handling
        def safe_min(arr):
            vals = [v for v in arr if v and not math.isinf(v) and not math.isnan(v)]
            return min(vals) if vals else float('inf')

        min_front = safe_min(front_ranges)
        min_left = safe_min(left_ranges)
        min_right = safe_min(right_ranges)

        self.front_clear = (min_front > self.min_dist)
        self.left_free = (min_left > self.side_dist)
        self.right_free = (min_right > self.side_dist)
        self.last_scan_time = self.get_clock().now()

        # debug
        # self.get_logger().debug(f'min_front={min_front:.2f} left={min_left:.2f} right={min_right:.2f}')

    def control_loop(self):
        # safety: ensure we have recent scan
        now = self.get_clock().now()
        if (now - self.last_scan_time) > Duration(seconds=1.0):
            # No recent scan -> stop
            self.publish_twist(0.0, 0.0)
            self.get_logger().warn('No recent LaserScan - stopping')
            return

        # Randomness to escape loops
        if random.random() < self.random_move_chance:
            # random short turn
            self.state = State.TURN
            self.turn_direction = random.choice([-1, 1])
            self.turn_end_time = now + Duration(seconds=random.uniform(self.turn_duration_min, self.turn_duration_max))
            self.get_logger().info('Random turn triggered')

        if self.state == State.FORWARD:
            if not self.front_clear:
                # need to turn - pick side with more clearance
                if self.left_free and not self.right_free:
                    self.turn_direction = 1
                elif self.right_free and not self.left_free:
                    self.turn_direction = -1
                else:
                    # both sides similar or blocked - choose the side with larger min distance
                    # we'll compare by checking tf (cheap) or random choice
                    self.turn_direction = random.choice([-1, 1])

                self.state = State.TURN
                self.turn_end_time = now + Duration(seconds=random.uniform(self.turn_duration_min, self.turn_duration_max))
                self.get_logger().info(f'Obstacle ahead - switching to TURN dir={self.turn_direction}')
            else:
                # move forward
                self.publish_twist(self.forward_speed, 0.0)

        elif self.state == State.TURN:
            # if front cleared before turn_end, go forward
            if self.front_clear and (self.turn_end_time is None or now >= self.turn_end_time):
                self.state = State.FORWARD
                self.get_logger().info('Front cleared - FORWARD')
                self.publish_twist(self.forward_speed, 0.0)
            else:
                # continue turning
                ang = float(self.turn_direction) * self.turn_speed
                self.publish_twist(0.0, ang)

        elif self.state == State.SPIRAL:
            # spiral behavior: forward slowly + slow rotation that changes over time
            t = time.time()
            # vary angular to create spiraling outward effect
            ang = math.copysign(self.spiral_turn_rate, math.sin(t * 0.4))
            self.publish_twist(self.spiral_forward, ang)
            # if obstacle ahead, switch to TURN
            if not self.front_clear:
                self.state = State.TURN
                self.turn_direction = -1 if random.random() < 0.5 else 1
                self.turn_end_time = now + Duration(seconds=random.uniform(self.turn_duration_min, self.turn_duration_max))
                self.get_logger().info('SPIRAL hit obstacle - TURN')

        else:
            # default safe stop
            self.publish_twist(0.0, 0.0)

    def publish_twist(self, linear_x: float, angular_z: float):
        t = Twist()
        t.linear.x = float(linear_x)
        t.angular.z = float(angular_z)
        self.cmd_pub.publish(t)

    # Optional helper: get robot pose via TF
    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time())
            # trans.transform.translation.x, y, z
            # trans.transform.rotation -> quaternion
            return trans
        except TransformException as ex:
            # Could not get transform
            # self.get_logger().debug(f'TF lookup failed: {ex}')
            return None


def main(args=None):
    rclpy.init(args=args)
    node = VacuumNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
