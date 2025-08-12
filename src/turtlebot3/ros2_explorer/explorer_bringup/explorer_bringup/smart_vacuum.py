#!/usr/bin/env python3
"""
vacuum_navigator_no_odom.py

Reactive vacuum navigator that DOES NOT use /odom or TF.
Relies only on LaserScan (/scan) and publishes /cmd_vel.

Behavior:
- FORWARD: move forward when front clear
- TURN: rotate in chosen direction when obstacle ahead
- SPIRAL: spiral escape when stuck
- Wall-follow bias: gently prefer the side with more free space
- Stuck detection: based on scan-change history (if robot keeps seeing same near obstacle while
  trying to move, consider stuck)
"""

import math
import random
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class State(Enum):
    FORWARD = 1
    TURN = 2
    SPIRAL = 3


class VacuumNavigatorNoOdom(Node):
    def __init__(self):
        super().__init__('vacuum_navigator_no_odom')

        # Parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('forward_speed', 0.15)
        self.declare_parameter('turn_speed', 0.7)
        self.declare_parameter('min_dist', 0.35)
        self.declare_parameter('side_dist', 0.45)
        self.declare_parameter('front_angle', 20.0)   # degrees for front sector
        self.declare_parameter('stuck_check_interval', 1.5)  # seconds
        self.declare_parameter('stuck_threshold', 4)  # how many checks of 'no progress' -> stuck
        self.declare_parameter('random_move_chance', 0.015)
        self.declare_parameter('wall_follow_bias', 0.22)  # small angular bias toward wide side
        self.declare_parameter('spiral_linear', 0.08)
        self.declare_parameter('spiral_angular', 0.45)
        self.declare_parameter('turn_cooldown', 1.0)  # after a turn, prefer not to immediately reverse

        # Load params
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.min_dist = self.get_parameter('min_dist').value
        self.side_dist = self.get_parameter('side_dist').value
        self.front_angle = self.get_parameter('front_angle').value
        self.stuck_check_interval = self.get_parameter('stuck_check_interval').value
        self.stuck_threshold = int(self.get_parameter('stuck_threshold').value)
        self.random_move_chance = self.get_parameter('random_move_chance').value
        self.wall_follow_bias = self.get_parameter('wall_follow_bias').value
        self.spiral_linear = self.get_parameter('spiral_linear').value
        self.spiral_angular = self.get_parameter('spiral_angular').value
        self.turn_cooldown = self.get_parameter('turn_cooldown').value

        # Pub/Sub
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        # State
        self.state = State.FORWARD
        self.last_scan_time = time.time()
        self.front_clear = True
        self.left_free = True
        self.right_free = True
        self.turn_direction = 1
        self.turn_end_time = None
        self.last_turn_time = 0.0
        self.last_turn_dir = 0

        # For stuck detection (no odom): compare front-sector median over time while trying to move
        self.last_front_value = None
        self.last_stuck_check = time.time()
        self.stuck_no_progress_count = 0

        # Timer loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('VacuumNavigatorNoOdom started')

    def scan_callback(self, msg: LaserScan):
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        n = len(ranges)

        # convert degrees to index helper
        def idx(deg):
            rad = math.radians(deg)
            return max(0, min(n - 1, int((rad - angle_min) / angle_inc)))

        # front sector centered at 0: -front_angle .. +front_angle
        left_idx = idx(self.front_angle)
        right_idx = idx(-self.front_angle)
        if right_idx <= left_idx:
            front_sector = ranges[right_idx:left_idx + 1]
        else:
            # wrap-around
            front_sector = ranges[right_idx:] + ranges[:left_idx + 1]

        # left sector: +30 .. +110 deg
        li = idx(30)
        le = idx(110)
        left_sector = ranges[li:le + 1] if li <= le else ranges[li:] + ranges[:le + 1]

        # right sector: -110 .. -30 deg
        ri = idx(-110)
        re = idx(-30)
        right_sector = ranges[ri:re + 1] if ri <= re else ranges[ri:] + ranges[:re + 1]

        def safe_stats(arr):
            vals = [v for v in arr if v and not math.isinf(v) and not math.isnan(v)]
            if not vals:
                return float('inf'), float('inf'), float('inf')
            vals_sorted = sorted(vals)
            median = vals_sorted[len(vals_sorted) // 2]
            mn = min(vals_sorted)
            avg = sum(vals_sorted) / len(vals_sorted)
            return median, mn, avg

        f_med, f_min, f_avg = safe_stats(front_sector)
        l_med, l_min, l_avg = safe_stats(left_sector)
        r_med, r_min, r_avg = safe_stats(right_sector)

        self.front_clear = (f_min > self.min_dist)
        self.left_free = (l_med > self.side_dist)
        self.right_free = (r_med > self.side_dist)
        self.last_scan_time = time.time()

        # store front metric for stuck detection
        self.current_front_metric = f_med

        # debug occasionally
        # self.get_logger().debug(f'front_med={f_med:.2f} l_med={l_med:.2f} r_med={r_med:.2f}')

    def control_loop(self):
        now = time.time()

        # safety stop if no scan recently
        if now - self.last_scan_time > 1.0:
            self.publish_twist(0.0, 0.0)
            self.get_logger().warn('No recent LaserScan - stopping')
            return

        # small randomness to escape loops
        if random.random() < self.random_move_chance and self.state == State.FORWARD:
            # short random turn
            self.state = State.TURN
            self.turn_direction = random.choice([-1, 1])
            self.turn_end_time = now + random.uniform(0.5, 1.4)
            self.last_turn_time = now
            self.last_turn_dir = self.turn_direction
            self.get_logger().info('Random short turn triggered')

        # Stuck detection: check every stuck_check_interval seconds
        if now - self.last_stuck_check >= self.stuck_check_interval:
            if hasattr(self, 'current_front_metric') and self.current_front_metric is not None:
                if self.last_front_value is not None:
                    # if we have been trying to move (state FORWARD recently) but front metric did not increase,
                    # assume not making progress (stuck)
                    if self.state == State.FORWARD:
                        # if front median remains small (< min_dist*1.2) and hasn't improved
                        if self.current_front_metric < (self.min_dist * 1.2) and abs(self.current_front_metric - self.last_front_value) < 0.03:
                            self.stuck_no_progress_count += 1
                        else:
                            # progress or free, reset counter
                            self.stuck_no_progress_count = max(0, self.stuck_no_progress_count - 1)
                # update last_front_value
                self.last_front_value = self.current_front_metric
            self.last_stuck_check = now

        # if stuck detected many times -> spiral to escape
        if self.stuck_no_progress_count >= self.stuck_threshold:
            self.get_logger().info('Detected stuck (no progress). Doing spiral escape.')
            self.state = State.SPIRAL
            self.stuck_no_progress_count = 0
            # set a brief spiral duration
            self.spiral_end_time = now + 1.6

        # State machine
        if self.state == State.FORWARD:
            if not self.front_clear:
                # choose turn direction: prefer side with more free space
                if self.left_free and not self.right_free:
                    self.turn_direction = 1
                elif self.right_free and not self.left_free:
                    self.turn_direction = -1
                else:
                    # if both similar, choose not to reverse direction immediately if just turned
                    if now - self.last_turn_time < self.turn_cooldown and self.last_turn_dir != 0:
                        self.turn_direction = -self.last_turn_dir
                    else:
                        self.turn_direction = random.choice([-1, 1])
                self.state = State.TURN
                self.turn_end_time = now + random.uniform(0.6, 1.4)
                self.last_turn_time = now
                self.last_turn_dir = self.turn_direction
                self.get_logger().info(f'Obstacle ahead - switching to TURN dir={self.turn_direction}')
                self.publish_twist(0.0, self.turn_direction * self.turn_speed)
            else:
                # forward with small bias toward wide side (wall-follow tendency)
                bias = 0.0
                # if left much wider, veer left (positive angular), if right wider, veer right
                # note: positive angular = CCW (left)
                if hasattr(self, 'current_front_metric'):
                    # We use left_free/right_free booleans plus random small bias magnitude
                    if self.left_free and not self.right_free:
                        bias = self.wall_follow_bias
                    elif self.right_free and not self.left_free:
                        bias = -self.wall_follow_bias
                # publish forward with bias
                self.publish_twist(self.forward_speed, bias)

        elif self.state == State.TURN:
            # continue turning until time expired or front becomes clear
            if self.front_clear and (self.turn_end_time is None or now >= self.turn_end_time):
                self.state = State.FORWARD
                self.get_logger().info('Front cleared - FORWARD')
                self.publish_twist(self.forward_speed, 0.0)
            else:
                # keep turning
                self.publish_twist(0.0, float(self.turn_direction) * self.turn_speed)

        elif self.state == State.SPIRAL:
            # spiral motion until spiral_end_time
            if hasattr(self, 'spiral_end_time') and now < self.spiral_end_time:
                # spiral: forward + rotation
                ang = math.copysign(self.spiral_angular, math.sin(now * 0.7))
                self.publish_twist(self.spiral_linear, ang)
            else:
                # after spiral, resume turning to pick a different direction
                self.state = State.TURN
                self.turn_direction = random.choice([-1, 1])
                self.turn_end_time = now + random.uniform(0.7, 1.3)
                self.last_turn_time = now
                self.last_turn_dir = self.turn_direction
                self.get_logger().info('Spiral finished - switching to TURN')

        else:
            # fallback safe stop
            self.publish_twist(0.0, 0.0)

    def publish_twist(self, linear_x: float, angular_z: float):
        t = Twist()
        t.linear.x = float(linear_x)
        t.angular.z = float(angular_z)
        self.cmd_pub.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = VacuumNavigatorNoOdom()
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
