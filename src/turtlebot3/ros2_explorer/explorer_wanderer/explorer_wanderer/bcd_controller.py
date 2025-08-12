#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Controller cho robot thực hiện đường đi bao phủ BCD
Điều khiển robot di chuyển theo đường đi bao phủ đã được tạo bởi BCD algorithm
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray
import numpy as np
import math
import time

class BCDController(Node):
    """
    Controller cho robot thực hiện đường đi bao phủ BCD
    """
    
    def __init__(self):
        super().__init__('bcd_controller')
        
        # Action client cho navigation
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscribers
        self.coverage_path_subscription = self.create_subscription(
            MarkerArray, 'coverage_path', self.coverage_path_callback, 10
        )
        self.scan_subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )
        
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Parameters
        self.robot_radius = 0.1
        self.safety_distance = 0.3
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.position_tolerance = 0.1
        self.orientation_tolerance = 0.1
        
        # State variables
        self.coverage_path = []
        self.current_path_index = 0
        self.is_executing = False
        self.current_pose = None
        self.obstacle_detected = False
        
        # Timer cho control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("BCD Controller initialized")
    
    def coverage_path_callback(self, msg):
        """
        Callback nhận đường đi bao phủ từ BCD algorithm
        """
        self.coverage_path = []
        
        for marker in msg.markers:
            if marker.type == Marker.ARROW and len(marker.points) >= 2:
                segment = {
                    'start': (marker.points[0].x, marker.points[0].y),
                    'end': (marker.points[1].x, marker.points[1].y),
                    'direction': 'forward' if marker.color.r == 0.0 else 'transition'
                }
                self.coverage_path.append(segment)
        
        self.get_logger().info(f"Received coverage path with {len(self.coverage_path)} segments")
        
        # Bắt đầu thực hiện đường đi nếu chưa đang thực hiện
        if not self.is_executing and self.coverage_path:
            self.start_coverage_execution()
    
    def scan_callback(self, msg):
        """
        Callback xử lý dữ liệu laser scan
        """
        # Kiểm tra vật cản
        min_distance = min(msg.ranges)
        self.obstacle_detected = min_distance < self.safety_distance
        
        if self.obstacle_detected:
            self.get_logger().warn(f"Obstacle detected at distance: {min_distance:.2f}m")
    
    def start_coverage_execution(self):
        """
        Bắt đầu thực hiện đường đi bao phủ
        """
        if not self.coverage_path:
            self.get_logger().warn("No coverage path available")
            return
        
        self.current_path_index = 0
        self.is_executing = True
        self.get_logger().info("Starting BCD coverage execution")
    
    def control_loop(self):
        """
        Control loop chính cho việc điều khiển robot
        """
        if not self.is_executing or not self.coverage_path:
            return
        
        # Kiểm tra nếu có vật cản
        if self.obstacle_detected:
            self.stop_robot()
            return
        
        # Thực hiện segment hiện tại
        self.execute_current_segment()
    
    def execute_current_segment(self):
        """
        Thực hiện segment hiện tại trong đường đi bao phủ
        """
        if self.current_path_index >= len(self.coverage_path):
            self.get_logger().info("Coverage execution completed")
            self.is_executing = False
            return
        
        current_segment = self.coverage_path[self.current_path_index]
        
        # Tính toán hướng di chuyển
        start_point = current_segment['start']
        end_point = current_segment['end']
        
        # Tính toán góc hướng
        dx = end_point[0] - start_point[0]
        dy = end_point[1] - start_point[1]
        target_angle = math.atan2(dy, dx)
        
        # Điều khiển robot
        self.move_robot_to_target(end_point, target_angle)
    
    def move_robot_to_target(self, target_point, target_angle):
        """
        Di chuyển robot đến điểm đích
        """
        # Tạo goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = target_point[0]
        goal_msg.pose.pose.position.y = target_point[1]
        goal_msg.pose.pose.position.z = 0.0
        
        # Chuyển đổi góc thành quaternion
        goal_msg.pose.pose.orientation.w = math.cos(target_angle / 2)
        goal_msg.pose.pose.orientation.z = math.sin(target_angle / 2)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        
        # Gửi goal
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """
        Callback khi nhận response từ action server
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Navigation goal rejected")
            return
        
        self.get_logger().info("Navigation goal accepted")
        
        # Lấy result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """
        Callback khi nhận result từ action server
        """
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Reached target point {self.current_path_index + 1}")
            self.current_path_index += 1
        else:
            self.get_logger().warn(f"Navigation failed with status: {status}")
    
    def stop_robot(self):
        """
        Dừng robot khi phát hiện vật cản
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_publisher.publish(cmd_vel)
    
    def emergency_stop(self):
        """
        Dừng khẩn cấp
        """
        self.stop_robot()
        self.is_executing = False
        self.get_logger().warn("Emergency stop activated")


class BCDCoverageExecutor(Node):
    """
    Executor chính cho việc thực hiện coverage BCD
    """
    
    def __init__(self):
        super().__init__('bcd_coverage_executor')
        
        # Tạo BCD coverage node
        self.bcd_coverage = BCDCoverage()
        
        # Tạo controller
        self.controller = BCDController()
        
        # Timer để kiểm tra trạng thái
        self.status_timer = self.create_timer(1.0, self.status_check)
        
        self.get_logger().info("BCD Coverage Executor initialized")
    
    def status_check(self):
        """
        Kiểm tra trạng thái của hệ thống
        """
        if self.controller.coverage_path:
            self.get_logger().info(
                f"Coverage progress: {self.controller.current_path_index + 1}/{len(self.controller.coverage_path)}"
            )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Tạo executor
        executor = BCDCoverageExecutor()
        
        # Spin
        rclpy.spin(executor)
    except KeyboardInterrupt:
        executor.controller.emergency_stop()
    finally:
        executor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 