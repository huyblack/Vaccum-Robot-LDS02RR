#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import std_srvs.srv
import numpy as np
import heapq
import math
import random
import threading
import time
import sys
import os
from typing import List, Tuple, Optional

class FrontierExplorationNode(Node):
    """
    Frontier-Based Exploration Node cho TurtleBot3
    Tích hợp với SLAM Cartographer để khám phá môi trường chưa biết
    """
    
    def __init__(self):
        super().__init__('frontier_exploration')
        
        # Parameters
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('speed', 0.1)
        self.declare_parameter('expansion_size', 3)
        self.declare_parameter('target_error', 0.1)
        self.declare_parameter('robot_r', 0.2)
        self.declare_parameter('auto_start', False)
        
        # Load parameters
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.speed = self.get_parameter('speed').value
        self.expansion_size = self.get_parameter('expansion_size').value
        self.target_error = self.get_parameter('target_error').value
        self.robot_r = self.get_parameter('robot_r').value
        self.auto_start = self.get_parameter('auto_start').value
        
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State variables
        self.map_data = None
        self.odom_data = None
        self.scan_data = None
        self.exploration_active = self.auto_start  # Chỉ active nếu auto_start = True
        self.path_global = None
        self.current_path = None
        self.current_target_index = 0
        
        # Service for external control
        self.start_exploration_service = self.create_service(
            std_srvs.srv.Trigger, 'start_exploration', self.start_exploration_callback)
        self.stop_exploration_service = self.create_service(
            std_srvs.srv.Trigger, 'stop_exploration', self.stop_exploration_callback)
        
        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        # Map info
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_width = 0
        self.map_height = 0
        self.map_data_array = None
        
        # Exploration thread
        self.exploration_thread = threading.Thread(target=self.exploration_loop, daemon=True)
        self.exploration_thread.start()
        
        self.get_logger().info("🚀 Frontier Exploration Node đã khởi động")
        self.get_logger().info(f"📊 Parameters: speed={self.speed}, lookahead={self.lookahead_distance}")
        if self.auto_start:
            self.get_logger().info("🔄 Auto-start enabled - exploration will begin automatically")
        else:
            self.get_logger().info("⏸️ Auto-start disabled - waiting for start command")
    
    def start_exploration_callback(self, request, response):
        """Service callback để bắt đầu exploration"""
        self.get_logger().info(f"📞 Service call: start_exploration (current active: {self.exploration_active})")
        
        if not self.exploration_active:
            self.exploration_active = True
            self.get_logger().info("🚀 Frontier Exploration STARTED via service call")
            response.success = True
            response.message = "Frontier Exploration started successfully"
        else:
            self.get_logger().warn("⚠️ Exploration already active - ignoring start request")
            response.success = False
            response.message = "Exploration already active"
        return response
    
    def stop_exploration_callback(self, request, response):
        """Service callback để dừng exploration"""
        if self.exploration_active:
            self.exploration_active = False
            # Send stop command to robot
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info("🛑 Frontier Exploration STOPPED via service call")
            response.success = True
            response.message = "Frontier Exploration stopped successfully"
        else:
            response.success = False
            response.message = "Exploration already stopped"
        return response
    
    def euler_from_quaternion(self, x, y, z, w):
        """Chuyển đổi Quaternion sang Euler angles (yaw)"""
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return yaw_z
    
    def heuristic(self, a, b):
        """Heuristic function cho A* algorithm"""
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
    
    def astar(self, array, start, goal):
        """A* pathfinding algorithm"""
        neighbors = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]
        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: self.heuristic(start, goal)}
        oheap = []
        heapq.heappush(oheap, (fscore[start], start))
        
        while oheap:
            current = heapq.heappop(oheap)[1]
            
            if current == goal:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                data.append(start)
                data.reverse()
                return data
            
            close_set.add(current)
            
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = gscore[current] + self.heuristic(current, neighbor)
                
                if (0 <= neighbor[0] < array.shape[0] and 
                    0 <= neighbor[1] < array.shape[1] and 
                    array[neighbor[0]][neighbor[1]] != 1):
                    
                    if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                        continue
                    
                    if (tentative_g_score < gscore.get(neighbor, 0) or 
                        neighbor not in [i[1] for i in oheap]):
                        came_from[neighbor] = current
                        gscore[neighbor] = tentative_g_score
                        fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                        heapq.heappush(oheap, (fscore[neighbor], neighbor))
        
        # If no path found, return closest path
        if goal not in came_from:
            closest_node = None
            closest_dist = float('inf')
            for node in close_set:
                dist = self.heuristic(node, goal)
                if dist < closest_dist:
                    closest_node = node
                    closest_dist = dist
            
            if closest_node is not None:
                data = []
                while closest_node in came_from:
                    data.append(closest_node)
                    closest_node = came_from[closest_node]
                data.append(start)
                data.reverse()
                return data
        
        return None
    
    def frontier_detection(self, occupancy_grid):
        """Tìm frontier points (biên giới giữa vùng đã biết và chưa biết)"""
        frontiers = []
        height, width = occupancy_grid.shape
        
        for i in range(1, height - 1):
            for j in range(1, width - 1):
                if occupancy_grid[i, j] == 0:  # Free space
                    # Check if adjacent to unknown area
                    if (occupancy_grid[i-1, j] == -1 or occupancy_grid[i+1, j] == -1 or
                        occupancy_grid[i, j-1] == -1 or occupancy_grid[i, j+1] == -1):
                        frontiers.append((i, j))
        
        return frontiers
    
    def group_frontiers(self, frontiers):
        """Nhóm các frontier points thành clusters"""
        if not frontiers:
            return []
        
        # Simple clustering based on distance
        clusters = []
        visited = set()
        
        for frontier in frontiers:
            if frontier in visited:
                continue
            
            cluster = [frontier]
            visited.add(frontier)
            
            # Find nearby frontiers
            for other in frontiers:
                if other not in visited:
                    dist = self.heuristic(frontier, other)
                    if dist < 5:  # Distance threshold
                        cluster.append(other)
                        visited.add(other)
            
            if len(cluster) > 2:  # Minimum cluster size
                clusters.append(cluster)
        
        return clusters
    
    def find_best_frontier(self, occupancy_grid, robot_pos):
        """Tìm frontier tốt nhất để khám phá"""
        frontiers = self.frontier_detection(occupancy_grid)
        if not frontiers:
            return None
        
        clusters = self.group_frontiers(frontiers)
        if not clusters:
            return None
        
        # Find cluster with best score (size / distance)
        best_cluster = None
        best_score = -1
        
        for cluster in clusters:
            # Calculate cluster centroid
            centroid_x = sum(p[0] for p in cluster) / len(cluster)
            centroid_y = sum(p[1] for p in cluster) / len(cluster)
            centroid = (int(centroid_x), int(centroid_y))
            
            # Calculate path to centroid
            path = self.astar(occupancy_grid, robot_pos, centroid)
            if path:
                distance = len(path)
                score = len(cluster) / distance
                
                if score > best_score:
                    best_score = score
                    best_cluster = cluster
        
        if best_cluster:
            # Return centroid of best cluster
            centroid_x = sum(p[0] for p in best_cluster) / len(best_cluster)
            centroid_y = sum(p[1] for p in best_cluster) / len(best_cluster)
            return (int(centroid_x), int(centroid_y))
        
        return None
    
    def pure_pursuit(self, current_x, current_y, current_heading, path, index):
        """Pure Pursuit controller"""
        closest_point = None
        v = self.speed
        
        for i in range(index, len(path)):
            x = path[i][0]
            y = path[i][1]
            distance = math.hypot(current_x - x, current_y - y)
            
            if self.lookahead_distance < distance:
                closest_point = (x, y)
                index = i
                break
        
        if closest_point is not None:
            target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
            desired_steering_angle = target_heading - current_heading
        else:
            target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
            desired_steering_angle = target_heading - current_heading
            index = len(path) - 1
        
        # Normalize angle
        if desired_steering_angle > math.pi:
            desired_steering_angle -= 2 * math.pi
        elif desired_steering_angle < -math.pi:
            desired_steering_angle += 2 * math.pi
        
        # Limit steering angle
        if desired_steering_angle > math.pi/6 or desired_steering_angle < -math.pi/6:
            sign = 1 if desired_steering_angle > 0 else -1
            desired_steering_angle = sign * math.pi/4
            v = 0.0
        
        return v, desired_steering_angle, index
    
    def local_obstacle_avoidance(self, scan_ranges):
        """Local obstacle avoidance"""
        v = None
        w = None
        
        # Check front obstacles
        for i in range(60):
            if scan_ranges[i] < self.robot_r:
                v = 0.2
                w = -math.pi/4
                break
        
        if v is None:
            # Check back obstacles
            for i in range(300, 360):
                if scan_ranges[i] < self.robot_r:
                    v = 0.2
                    w = math.pi/4
                    break
        
        return v, w
    
    def exploration_loop(self):
        """Main exploration loop"""
        self.get_logger().info("🔄 Exploration loop started")
        
        while self.exploration_active:
            if (self.map_data is None or self.odom_data is None or 
                self.scan_data is None):
                self.get_logger().debug("⏳ Waiting for sensor data...")
                time.sleep(0.1)
                continue
            
            try:
                # Convert robot position to grid coordinates
                robot_col = int((self.robot_x - self.map_origin_x) / self.map_resolution)
                robot_row = int((self.robot_y - self.map_origin_y) / self.map_resolution)
                
                self.get_logger().info(f"🤖 Robot position: World=({self.robot_x:.2f}, {self.robot_y:.2f}), Grid=({robot_row}, {robot_col})")
                self.get_logger().info(f"🗺️ Map info: size=({self.map_width}x{self.map_height}), origin=({self.map_origin_x:.2f}, {self.map_origin_y:.2f}), resolution={self.map_resolution}")
                
                # Check bounds
                if (robot_row < 0 or robot_row >= self.map_height or
                    robot_col < 0 or robot_col >= self.map_width):
                    self.get_logger().warn(f"⚠️ Robot position out of map bounds: Grid({robot_row}, {robot_col}) not in [0, {self.map_width})x[0, {self.map_height})")
                    time.sleep(0.1)
                    continue
                
                # Find best frontier
                target = self.find_best_frontier(self.map_data_array, (robot_row, robot_col))
                
                if target is None:
                    self.get_logger().info("🎉 Exploration completed - no more frontiers!")
                    self.exploration_active = False
                    break
                
                self.get_logger().info(f"🎯 Found target frontier: Grid{target}")
                
                # Plan path to target
                path = self.astar(self.map_data_array, (robot_row, robot_col), target)
                
                if path is None:
                    self.get_logger().warn("⚠️ No path to target found")
                    time.sleep(0.1)
                    continue
                
                self.get_logger().info(f"🛤️ Path planned: {len(path)} waypoints")
                
                # Convert path to world coordinates
                world_path = []
                for p in path:
                    world_x = p[1] * self.map_resolution + self.map_origin_x
                    world_y = p[0] * self.map_resolution + self.map_origin_y
                    world_path.append((world_x, world_y))
                
                # Follow path
                self.get_logger().info(f"🚀 Starting to follow path with {len(world_path)} waypoints")
                self.follow_path(world_path)
                
            except Exception as e:
                self.get_logger().error(f"❌ Exploration error: {e}")
                time.sleep(0.1)
    
    def follow_path(self, path):
        """Follow planned path using Pure Pursuit"""
        self.current_path = path
        self.current_target_index = 0
        
        self.get_logger().info(f"🎮 Following path: {len(path)} waypoints")
        
        while self.current_target_index < len(path):
            if not self.exploration_active:
                self.get_logger().info("🛑 Exploration stopped during path following")
                break
            
            # Check if reached target
            target = path[-1]
            distance_to_target = math.hypot(self.robot_x - target[0], self.robot_y - target[1])
            
            if distance_to_target < self.target_error:
                self.get_logger().info(f"✅ Reached target: ({target[0]:.2f}, {target[1]:.2f})")
                break
            
            # Local obstacle avoidance
            v, w = self.local_obstacle_avoidance(self.scan_data.ranges)
            
            if v is None:
                # Pure pursuit control
                v, w, self.current_target_index = self.pure_pursuit(
                    self.robot_x, self.robot_y, self.robot_yaw, path, self.current_target_index)
            
            # Publish velocity command
            twist = Twist()
            twist.linear.x = v
            twist.angular.z = w
            self.cmd_vel_pub.publish(twist)
            
            self.get_logger().info(f"🎮 Velocity command: linear={v:.3f} m/s, angular={w:.3f} rad/s")
            
            time.sleep(0.1)
    
    def map_callback(self, msg):
        """Map data callback"""
        self.map_data = msg
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        
        # Convert to numpy array
        self.map_data_array = np.array(msg.data).reshape(self.map_height, self.map_width)
    
    def odom_callback(self, msg):
        """Odometry callback"""
        self.odom_data = msg
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = self.euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
    
    def scan_callback(self, msg):
        """Laser scan callback"""
        self.scan_data = msg

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Frontier Exploration stopped by user")
    finally:
        node.exploration_active = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 