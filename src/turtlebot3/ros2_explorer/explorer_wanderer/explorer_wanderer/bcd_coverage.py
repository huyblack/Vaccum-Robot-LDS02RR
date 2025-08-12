#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Thuật toán Boustrophedon Cellular Decomposition (BCD) cho robot hút bụi
Triển khai thuật toán phân ô và đường đi bao phủ theo mô hình boustrophedon
"""

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import math
from collections import defaultdict
import cv2

class BCDCoverage(Node):
    """
    Thuật toán Boustrophedon Cellular Decomposition cho robot hút bụi
    """
    
    def __init__(self):
        super().__init__('bcd_coverage')
        
        # Subscribers
        self.map_subscription = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10
        )
        
        # Publishers
        self.cells_publisher = self.create_publisher(
            MarkerArray, 'bcd_cells', 10
        )
        self.coverage_path_publisher = self.create_publisher(
            MarkerArray, 'coverage_path', 10
        )
        
        # Parameters
        self.robot_radius = 0.1  # Bán kính robot (m)
        self.coverage_spacing = 0.15  # Khoảng cách giữa các đường bao phủ (m)
        self.wall_distance = 0.05  # Khoảng cách với tường (m)
        
        # BCD variables
        self.cells = []  # Danh sách các ô
        self.adjacency_graph = defaultdict(list)  # Đồ thị kề cận
        self.coverage_path = []  # Đường đi bao phủ
        self.current_map = None
        self.map_resolution = 0.05
        self.map_width = 0
        self.map_height = 0
        self.map_origin = None
        
        self.get_logger().info("BCD Coverage algorithm initialized")
    
    def map_callback(self, msg):
        """Callback xử lý bản đồ occupancy grid"""
        self.current_map = msg
        self.map_resolution = msg.info.resolution
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = msg.info.origin
        
        # Thực hiện BCD decomposition
        self.perform_bcd_decomposition()
        
        # Tạo đường đi bao phủ
        self.generate_coverage_path()
        
        # Publish visualization
        self.publish_cells_visualization()
        self.publish_coverage_path_visualization()
    
    def perform_bcd_decomposition(self):
        """
        Thực hiện thuật toán Boustrophedon Cellular Decomposition
        """
        if self.current_map is None:
            return
            
        self.get_logger().info("Performing BCD decomposition...")
        
        # Chuyển đổi occupancy grid thành numpy array
        map_data = np.array(self.current_map.data).reshape(
            self.map_height, self.map_width
        )
        
        # Tìm các đỉnh của vật cản
        obstacle_vertices = self.find_obstacle_vertices(map_data)
        
        # Tạo các đường phân chia dọc theo trục Y
        vertical_lines = self.create_vertical_decomposition_lines(obstacle_vertices)
        
        # Phân chia thành các ô
        self.cells = self.decompose_into_cells(map_data, vertical_lines)
        
        # Tạo đồ thị kề cận
        self.adjacency_graph = self.build_adjacency_graph()
        
        self.get_logger().info(f"BCD decomposition completed: {len(self.cells)} cells")
    
    def find_obstacle_vertices(self, map_data):
        """
        Tìm các đỉnh của vật cản trong bản đồ
        """
        vertices = []
        
        # Tìm các pixel occupied (vật cản)
        occupied_pixels = np.where(map_data > 50)
        
        # Sử dụng contour detection để tìm đỉnh
        for i in range(len(occupied_pixels[0])):
            y, x = occupied_pixels[0][i], occupied_pixels[1][i]
            
            # Kiểm tra xem có phải đỉnh không
            if self.is_vertex(map_data, x, y):
                # Chuyển đổi từ pixel coordinates sang world coordinates
                world_x = x * self.map_resolution + self.map_origin.position.x
                world_y = y * self.map_resolution + self.map_origin.position.y
                vertices.append((world_x, world_y))
        
        return vertices
    
    def is_vertex(self, map_data, x, y):
        """
        Kiểm tra xem pixel có phải là đỉnh của vật cản không
        """
        if x <= 0 or x >= self.map_width - 1 or y <= 0 or y >= self.map_height - 1:
            return False
            
        # Kiểm tra 8 neighbors
        neighbors = [
            map_data[y-1, x-1], map_data[y-1, x], map_data[y-1, x+1],
            map_data[y, x-1], map_data[y, x+1],
            map_data[y+1, x-1], map_data[y+1, x], map_data[y+1, x+1]
        ]
        
        # Đếm số neighbors occupied
        occupied_count = sum(1 for n in neighbors if n > 50)
        
        # Nếu có ít hơn 4 neighbors occupied, có thể là đỉnh
        return occupied_count < 4
    
    def create_vertical_decomposition_lines(self, vertices):
        """
        Tạo các đường phân chia dọc theo trục Y
        """
        lines = []
        
        for vertex in vertices:
            x, y = vertex
            
            # Tạo đường thẳng dọc từ đỉnh
            line = {
                'x': x,
                'start_y': self.map_origin.position.y,
                'end_y': self.map_origin.position.y + self.map_height * self.map_resolution
            }
            lines.append(line)
        
        # Sắp xếp theo x coordinate
        lines.sort(key=lambda l: l['x'])
        
        return lines
    
    def decompose_into_cells(self, map_data, vertical_lines):
        """
        Phân chia bản đồ thành các ô dựa trên các đường phân chia
        """
        cells = []
        
        # Tạo các ô giữa các đường phân chia
        for i in range(len(vertical_lines) - 1):
            left_line = vertical_lines[i]
            right_line = vertical_lines[i + 1]
            
            cell = {
                'id': i,
                'left_boundary': left_line['x'],
                'right_boundary': right_line['x'],
                'top_boundary': self.map_origin.position.y + self.map_height * self.map_resolution,
                'bottom_boundary': self.map_origin.position.y,
                'coverage_lines': []
            }
            
            # Tạo các đường bao phủ trong ô
            cell['coverage_lines'] = self.generate_coverage_lines_for_cell(cell, map_data)
            
            cells.append(cell)
        
        return cells
    
    def generate_coverage_lines_for_cell(self, cell, map_data):
        """
        Tạo các đường bao phủ trong một ô
        """
        coverage_lines = []
        
        # Bắt đầu từ bottom boundary
        current_y = cell['bottom_boundary'] + self.wall_distance
        
        while current_y < cell['top_boundary'] - self.wall_distance:
            # Tạo đường bao phủ ngang
            line = {
                'start_x': cell['left_boundary'] + self.wall_distance,
                'end_x': cell['right_boundary'] - self.wall_distance,
                'y': current_y,
                'direction': 'right'  # Bắt đầu từ trái sang phải
            }
            
            coverage_lines.append(line)
            
            # Di chuyển lên trên với khoảng cách coverage_spacing
            current_y += self.coverage_spacing
            
            # Nếu còn chỗ, tạo đường ngược lại
            if current_y < cell['top_boundary'] - self.wall_distance:
                line = {
                    'start_x': cell['right_boundary'] - self.wall_distance,
                    'end_x': cell['left_boundary'] + self.wall_distance,
                    'y': current_y,
                    'direction': 'left'  # Từ phải sang trái
                }
                
                coverage_lines.append(line)
                current_y += self.coverage_spacing
        
        return coverage_lines
    
    def build_adjacency_graph(self):
        """
        Xây dựng đồ thị kề cận giữa các ô
        """
        graph = defaultdict(list)
        
        for i, cell1 in enumerate(self.cells):
            for j, cell2 in enumerate(self.cells):
                if i != j:
                    # Kiểm tra xem hai ô có kề nhau không
                    if self.are_cells_adjacent(cell1, cell2):
                        graph[i].append(j)
        
        return graph
    
    def are_cells_adjacent(self, cell1, cell2):
        """
        Kiểm tra xem hai ô có kề nhau không
        """
        # Hai ô kề nhau nếu có chung boundary
        if (abs(cell1['right_boundary'] - cell2['left_boundary']) < 0.1 or
            abs(cell1['left_boundary'] - cell2['right_boundary']) < 0.1):
            return True
        
        return False
    
    def generate_coverage_path(self):
        """
        Tạo đường đi bao phủ toàn bộ môi trường
        """
        if not self.cells:
            return
        
        self.coverage_path = []
        
        # Sử dụng DFS để tìm đường đi qua tất cả các ô
        visited = set()
        path = []
        
        def dfs(cell_id):
            visited.add(cell_id)
            path.append(cell_id)
            
            # Thêm đường bao phủ của ô hiện tại
            cell = self.cells[cell_id]
            for line in cell['coverage_lines']:
                self.coverage_path.append({
                    'cell_id': cell_id,
                    'start': (line['start_x'], line['y']),
                    'end': (line['end_x'], line['y']),
                    'direction': line['direction']
                })
            
            # Thăm các ô kề cận
            for neighbor in self.adjacency_graph[cell_id]:
                if neighbor not in visited:
                    # Thêm đường chuyển tiếp giữa các ô
                    self.add_transition_path(cell_id, neighbor)
                    dfs(neighbor)
        
        # Bắt đầu từ ô đầu tiên
        dfs(0)
        
        self.get_logger().info(f"Generated coverage path with {len(self.coverage_path)} segments")
    
    def add_transition_path(self, from_cell_id, to_cell_id):
        """
        Thêm đường chuyển tiếp giữa hai ô
        """
        from_cell = self.cells[from_cell_id]
        to_cell = self.cells[to_cell_id]
        
        # Tìm điểm chuyển tiếp
        if abs(from_cell['right_boundary'] - to_cell['left_boundary']) < 0.1:
            # Chuyển từ phải sang trái
            transition_start = (from_cell['right_boundary'] - self.wall_distance, 
                              from_cell['coverage_lines'][-1]['y'])
            transition_end = (to_cell['left_boundary'] + self.wall_distance,
                            to_cell['coverage_lines'][0]['y'])
        else:
            # Chuyển từ trái sang phải
            transition_start = (from_cell['left_boundary'] + self.wall_distance,
                              from_cell['coverage_lines'][-1]['y'])
            transition_end = (to_cell['right_boundary'] - self.wall_distance,
                            to_cell['coverage_lines'][0]['y'])
        
        self.coverage_path.append({
            'cell_id': -1,  # -1 cho transition
            'start': transition_start,
            'end': transition_end,
            'direction': 'transition'
        })
    
    def publish_cells_visualization(self):
        """
        Publish visualization cho các ô BCD
        """
        marker_array = MarkerArray()
        
        for i, cell in enumerate(self.cells):
            # Tạo marker cho boundary của ô
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "bcd_cells"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            
            # Định nghĩa boundary của ô
            points = [
                Point(x=cell['left_boundary'], y=cell['bottom_boundary'], z=0.0),
                Point(x=cell['right_boundary'], y=cell['bottom_boundary'], z=0.0),
                Point(x=cell['right_boundary'], y=cell['top_boundary'], z=0.0),
                Point(x=cell['left_boundary'], y=cell['top_boundary'], z=0.0),
                Point(x=cell['left_boundary'], y=cell['bottom_boundary'], z=0.0)
            ]
            
            marker.points = points
            marker.scale.x = 0.05  # Độ dày đường
            
            # Màu sắc
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
            
            marker_array.markers.append(marker)
        
        self.cells_publisher.publish(marker_array)
    
    def publish_coverage_path_visualization(self):
        """
        Publish visualization cho đường đi bao phủ
        """
        marker_array = MarkerArray()
        
        for i, segment in enumerate(self.coverage_path):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "coverage_path"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # Định nghĩa điểm bắt đầu và kết thúc
            start_point = Point(x=segment['start'][0], y=segment['start'][1], z=0.0)
            end_point = Point(x=segment['end'][0], y=segment['end'][1], z=0.0)
            
            marker.points = [start_point, end_point]
            marker.scale.x = 0.1  # Độ dày mũi tên
            marker.scale.y = 0.05  # Độ rộng mũi tên
            
            # Màu sắc khác nhau cho transition
            if segment['direction'] == 'transition':
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)  # Đỏ
            else:
                marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)  # Xanh
            
            marker_array.markers.append(marker)
        
        self.coverage_path_publisher.publish(marker_array)
    
    def get_coverage_path(self):
        """
        Trả về đường đi bao phủ hiện tại
        """
        return self.coverage_path
    
    def get_cells(self):
        """
        Trả về danh sách các ô BCD
        """
        return self.cells


def main(args=None):
    rclpy.init(args=args)
    
    bcd_coverage = BCDCoverage()
    
    try:
        rclpy.spin(bcd_coverage)
    except KeyboardInterrupt:
        pass
    finally:
        bcd_coverage.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 