import numpy as np
import math
from typing import List, Tuple, Optional, Dict
from enum import Enum

class BoundaryWalkState(Enum):
    """Các trạng thái của thuật toán Boundary Walk"""
    FIND_WALL = "find_wall"           # Tìm tường để bắt đầu
    ALIGN_TO_WALL = "align_to_wall"   # Căn chỉnh song song với tường
    FOLLOW_WALL = "follow_wall"       # Đi theo tường
    CORNER_TURN = "corner_turn"       # Xử lý góc cua
    COMPLETED = "completed"           # Hoàn thành

class ZigZagState(Enum):
    """Các trạng thái của thuật toán Zig-zag"""
    STRAIGHT_PASS = "straight_pass"   # Đi thẳng một pass
    TURN_AROUND = "turn_around"       # Quay đầu để chuyển pass
    REPOSITIONING = "repositioning"   # Di chuyển đến vị trí tiếp theo
    COMPLETED = "completed"           # Hoàn thành

class BoundaryWalkAlgorithm:
    """
    Thuật toán Boundary Walk cho robot hút bụi
    Dựa trên mô tả trong README.md với FSM (Finite State Machine)
    """
    
    def __init__(self, wall_distance=0.25, robot_radius=0.2):
        self.wall_distance = wall_distance
        self.robot_radius = robot_radius
        self.state = BoundaryWalkState.FIND_WALL
        
        # Tracking variables
        self.followed_boundaries = set()
        self.start_position = None
        self.last_wall_side = 'right'  # 'left' hoặc 'right'
        self.turn_direction = 1  # 1 = right, -1 = left
        
        # State timeouts để tránh stuck
        self.state_timeout = 10.0  # seconds
        self.state_start_time = None
        
        print("🚶 [INFO] Boundary Walk Algorithm initialized")
    
    def get_next_target(self, occupancy_grid: np.ndarray, robot_pose: Tuple[float, float, float],
                       scan_ranges: List[float], grid_resolution: float,
                       origin_x: float, origin_y: float) -> Optional[Tuple[float, float]]:
        """
        Lấy target tiếp theo cho Boundary Walk
        """
        current_time = self._get_current_time()
        
        # Timeout check
        if self.state_start_time and (current_time - self.state_start_time) > self.state_timeout:
            print(f"⚠️  [WARNING] Boundary walk state timeout: {self.state}")
            self._transition_to_next_state()
        
        if self.state == BoundaryWalkState.FIND_WALL:
            return self._find_wall_target(occupancy_grid, robot_pose, scan_ranges, 
                                        grid_resolution, origin_x, origin_y)
        
        elif self.state == BoundaryWalkState.ALIGN_TO_WALL:
            return self._align_to_wall_target(robot_pose, scan_ranges)
        
        elif self.state == BoundaryWalkState.FOLLOW_WALL:
            return self._follow_wall_target(robot_pose, scan_ranges, grid_resolution, origin_x, origin_y)
        
        elif self.state == BoundaryWalkState.CORNER_TURN:
            return self._corner_turn_target(robot_pose, scan_ranges)
        
        else:  # COMPLETED
            return None
    
    def _find_wall_target(self, occupancy_grid: np.ndarray, robot_pose: Tuple[float, float, float],
                         scan_ranges: List[float], grid_resolution: float,
                         origin_x: float, origin_y: float) -> Optional[Tuple[float, float]]:
        """Tìm tường gần nhất để bắt đầu boundary walk"""
        
        # Tìm nearest wall từ laser scan
        min_distance = float('inf')
        wall_angle = 0
        
        for i, distance in enumerate(scan_ranges):
            if not math.isinf(distance) and not math.isnan(distance):
                if distance < min_distance and distance > self.robot_radius:
                    min_distance = distance
                    wall_angle = i * (2 * math.pi / len(scan_ranges))
        
        if min_distance < self.wall_distance * 3:  # Wall found nearby
            # Move towards wall
            target_x = robot_pose[0] + (min_distance - self.wall_distance) * math.cos(wall_angle + robot_pose[2])
            target_y = robot_pose[1] + (min_distance - self.wall_distance) * math.sin(wall_angle + robot_pose[2])
            
            if min_distance <= self.wall_distance * 1.5:
                self._transition_to_state(BoundaryWalkState.ALIGN_TO_WALL)
            
            return (target_x, target_y)
        else:
            # No wall nearby, explore to find one
            return self._explore_for_wall(occupancy_grid, robot_pose, grid_resolution, origin_x, origin_y)
    
    def _align_to_wall_target(self, robot_pose: Tuple[float, float, float],
                             scan_ranges: List[float]) -> Optional[Tuple[float, float]]:
        """Căn chỉnh robot song song với tường"""
        
        # Tìm hướng tường sử dụng TIDAL sensor simulation
        wall_direction = self._find_wall_direction(scan_ranges)
        
        if wall_direction is not None:
            # Calculate target để robot song song với tường
            target_heading = wall_direction + math.pi/2  # Perpendicular to wall
            
            # Move forward một chút để duy trì khoảng cách với tường
            target_distance = 0.3
            target_x = robot_pose[0] + target_distance * math.cos(target_heading)
            target_y = robot_pose[1] + target_distance * math.sin(target_heading)
            
            # Check if aligned enough
            heading_diff = abs(self._normalize_angle(target_heading - robot_pose[2]))
            if heading_diff < math.pi/12:  # ~15 degrees
                self._transition_to_state(BoundaryWalkState.FOLLOW_WALL)
            
            return (target_x, target_y)
        
        return None
    
    def _follow_wall_target(self, robot_pose: Tuple[float, float, float],
                           scan_ranges: List[float], grid_resolution: float,
                           origin_x: float, origin_y: float) -> Optional[Tuple[float, float]]:
        """Đi theo tường với khoảng cách cố định"""
        
        # Check for corners or obstacles ahead
        front_distance = self._get_front_distance(scan_ranges)
        side_distance = self._get_side_distance(scan_ranges, self.last_wall_side)
        
        if front_distance < self.wall_distance * 2:  # Obstacle ahead, need to turn
            self._transition_to_state(BoundaryWalkState.CORNER_TURN)
            return self._corner_turn_target(robot_pose, scan_ranges)
        
        # Maintain distance to wall
        wall_error = side_distance - self.wall_distance
        
        # Calculate next position along the wall
        forward_distance = 0.5
        lateral_correction = -wall_error * 0.5  # P controller
        
        # Current heading
        heading = robot_pose[2]
        
        # Target position
        target_x = robot_pose[0] + forward_distance * math.cos(heading) + \
                  lateral_correction * math.cos(heading + math.pi/2)
        target_y = robot_pose[1] + forward_distance * math.sin(heading) + \
                  lateral_correction * math.sin(heading + math.pi/2)
        
        # Mark this area as cleaned
        grid_x = int((robot_pose[0] - origin_x) / grid_resolution)
        grid_y = int((robot_pose[1] - origin_y) / grid_resolution)
        self.followed_boundaries.add((grid_x, grid_y))
        
        # Check completion (simple heuristic: returned to start area)
        if self.start_position is None:
            self.start_position = (robot_pose[0], robot_pose[1])
        elif self._distance_to_start(robot_pose) < 0.5 and len(self.followed_boundaries) > 50:
            self._transition_to_state(BoundaryWalkState.COMPLETED)
            return None
        
        return (target_x, target_y)
    
    def _corner_turn_target(self, robot_pose: Tuple[float, float, float],
                           scan_ranges: List[float]) -> Optional[Tuple[float, float]]:
        """Xử lý góc cua khi gặp corner"""
        
        # Perform 90-degree turn
        turn_angle = self.turn_direction * math.pi/2
        new_heading = robot_pose[2] + turn_angle
        
        # Move forward a bit after turning
        forward_distance = 0.3
        target_x = robot_pose[0] + forward_distance * math.cos(new_heading)
        target_y = robot_pose[1] + forward_distance * math.sin(new_heading)
        
        # Check if turn is complete and path is clear
        front_distance = self._get_front_distance(scan_ranges)
        if front_distance > self.wall_distance * 2:
            self._transition_to_state(BoundaryWalkState.FOLLOW_WALL)
            # Alternate turn direction for next corner
            self.turn_direction *= -1
        
        return (target_x, target_y)
    
    def _explore_for_wall(self, occupancy_grid: np.ndarray, robot_pose: Tuple[float, float, float],
                         grid_resolution: float, origin_x: float, origin_y: float) -> Optional[Tuple[float, float]]:
        """Khám phá để tìm tường"""
        
        # Simple exploration: move towards nearest known obstacle
        robot_col = int((robot_pose[0] - origin_x) / grid_resolution)
        robot_row = int((robot_pose[1] - origin_y) / grid_resolution)
        
        # Find nearest obstacle
        obstacle_positions = np.where(occupancy_grid == 100)
        if len(obstacle_positions[0]) > 0:
            distances = []
            for i in range(len(obstacle_positions[0])):
                obs_row, obs_col = obstacle_positions[0][i], obstacle_positions[1][i]
                dist = math.hypot(robot_row - obs_row, robot_col - obs_col)
                distances.append((dist, obs_row, obs_col))
            
            # Move towards closest obstacle
            distances.sort()
            target_row, target_col = distances[0][1], distances[0][2]
            
            target_x = target_col * grid_resolution + origin_x
            target_y = target_row * grid_resolution + origin_y
            
            return (target_x, target_y)
        
        return None
    
    def _find_wall_direction(self, scan_ranges: List[float]) -> Optional[float]:
        """Tìm hướng của tường sử dụng laser scan"""
        if not scan_ranges:
            return None
        
        # Find angle with minimum distance (wall direction)
        min_distance = float('inf')
        wall_angle = 0
        
        for i, distance in enumerate(scan_ranges):
            if not math.isinf(distance) and not math.isnan(distance):
                if distance < min_distance:
                    min_distance = distance
                    wall_angle = i * (2 * math.pi / len(scan_ranges))
        
        return wall_angle
    
    def _get_front_distance(self, scan_ranges: List[float]) -> float:
        """Lấy khoảng cách phía trước robot"""
        if not scan_ranges:
            return float('inf')
        
        # Front is typically around index 0 or len(scan_ranges)//2 depending on setup
        front_indices = list(range(len(scan_ranges)//2 - 30, len(scan_ranges)//2 + 30))
        front_distances = [scan_ranges[i] for i in front_indices if 0 <= i < len(scan_ranges)]
        
        valid_distances = [d for d in front_distances if not math.isinf(d) and not math.isnan(d)]
        return min(valid_distances) if valid_distances else float('inf')
    
    def _get_side_distance(self, scan_ranges: List[float], side: str) -> float:
        """Lấy khoảng cách bên cạnh robot"""
        if not scan_ranges:
            return float('inf')
        
        if side == 'right':
            side_indices = list(range(len(scan_ranges)*3//4 - 15, len(scan_ranges)*3//4 + 15))
        else:  # left
            side_indices = list(range(len(scan_ranges)//4 - 15, len(scan_ranges)//4 + 15))
        
        side_distances = [scan_ranges[i] for i in side_indices if 0 <= i < len(scan_ranges)]
        valid_distances = [d for d in side_distances if not math.isinf(d) and not math.isnan(d)]
        
        return min(valid_distances) if valid_distances else float('inf')
    
    def _transition_to_state(self, new_state: BoundaryWalkState):
        """Chuyển đổi sang state mới"""
        print(f"🚶 [BOUNDARY] Transition: {self.state.value} → {new_state.value}")
        self.state = new_state
        self.state_start_time = self._get_current_time()
    
    def _transition_to_next_state(self):
        """Chuyển sang state tiếp theo khi timeout"""
        if self.state == BoundaryWalkState.FIND_WALL:
            self._transition_to_state(BoundaryWalkState.COMPLETED)
        elif self.state == BoundaryWalkState.ALIGN_TO_WALL:
            self._transition_to_state(BoundaryWalkState.FOLLOW_WALL)
        elif self.state == BoundaryWalkState.FOLLOW_WALL:
            self._transition_to_state(BoundaryWalkState.CORNER_TURN)
        elif self.state == BoundaryWalkState.CORNER_TURN:
            self._transition_to_state(BoundaryWalkState.FOLLOW_WALL)
        else:
            self._transition_to_state(BoundaryWalkState.COMPLETED)
    
    def _distance_to_start(self, robot_pose: Tuple[float, float, float]) -> float:
        """Tính khoảng cách đến vị trí bắt đầu"""
        if self.start_position is None:
            return float('inf')
        return math.hypot(robot_pose[0] - self.start_position[0], 
                         robot_pose[1] - self.start_position[1])
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize góc về [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _get_current_time(self) -> float:
        """Get current time - stub for testing"""
        import time
        return time.time()
    
    def get_completion_rate(self) -> float:
        """Lấy tỷ lệ hoàn thành boundary walk"""
        if self.state == BoundaryWalkState.COMPLETED:
            return 1.0
        elif len(self.followed_boundaries) == 0:
            return 0.0
        else:
            # Estimate based on coverage and return to start
            coverage_factor = min(len(self.followed_boundaries) / 100, 0.8)
            if self.start_position:
                return_factor = 1.0 - min(self._distance_to_start((0, 0, 0)) / 5.0, 0.2)
                return coverage_factor + return_factor
            return coverage_factor


class ZigZagCoverageAlgorithm:
    """
    Thuật toán Zig-zag Coverage cho robot hút bụi
    Cover toàn bộ vùng free space với pattern zig-zag
    """
    
    def __init__(self, pass_width=0.4, robot_radius=0.2):
        self.pass_width = pass_width
        self.robot_radius = robot_radius
        self.state = ZigZagState.STRAIGHT_PASS
        
        # Planning variables
        self.coverage_region = None
        self.current_pass = 0
        self.total_passes = 0
        self.pass_direction = 1  # 1 = forward, -1 = backward
        self.covered_cells = set()
        
        # Current pass tracking
        self.pass_start_point = None
        self.pass_end_point = None
        self.current_pass_progress = 0.0
        
        print("🔄 [INFO] Zig-zag Coverage Algorithm initialized")
    
    def initialize_coverage_region(self, occupancy_grid: np.ndarray, 
                                  grid_resolution: float, origin_x: float, origin_y: float):
        """Khởi tạo vùng coverage và lập kế hoạch zig-zag passes"""
        
        # Tìm free space region
        free_space = (occupancy_grid == 0)
        rows, cols = np.where(free_space)
        
        if len(rows) == 0:
            print("⚠️  [WARNING] No free space found for zig-zag coverage")
            return
        
        # Calculate bounding box
        min_row, max_row = rows.min(), rows.max()
        min_col, max_col = cols.min(), cols.max()
        
        # Convert to world coordinates
        self.coverage_region = {
            'min_x': min_col * grid_resolution + origin_x,
            'max_x': max_col * grid_resolution + origin_x,
            'min_y': min_row * grid_resolution + origin_y,
            'max_y': max_row * grid_resolution + origin_y,
            'grid_resolution': grid_resolution,
            'origin_x': origin_x,
            'origin_y': origin_y
        }
        
        # Calculate number of passes needed
        coverage_width = self.coverage_region['max_x'] - self.coverage_region['min_x']
        self.total_passes = max(1, int(math.ceil(coverage_width / self.pass_width)))
        
        print(f"📋 [ZIGZAG] Planned {self.total_passes} passes for coverage")
        print(f"📐 [ZIGZAG] Coverage area: {coverage_width:.2f}m x {self.coverage_region['max_y'] - self.coverage_region['min_y']:.2f}m")
        
        # Start first pass
        self._start_new_pass()
    
    def get_next_target(self, robot_pose: Tuple[float, float, float],
                       occupancy_grid: np.ndarray) -> Optional[Tuple[float, float]]:
        """Lấy target tiếp theo cho zig-zag coverage"""
        
        if self.coverage_region is None:
            return None
        
        if self.state == ZigZagState.STRAIGHT_PASS:
            return self._straight_pass_target(robot_pose, occupancy_grid)
        
        elif self.state == ZigZagState.TURN_AROUND:
            return self._turn_around_target(robot_pose)
        
        elif self.state == ZigZagState.REPOSITIONING:
            return self._repositioning_target(robot_pose)
        
        else:  # COMPLETED
            return None
    
    def _straight_pass_target(self, robot_pose: Tuple[float, float, float],
                             occupancy_grid: np.ndarray) -> Optional[Tuple[float, float]]:
        """Target cho straight pass"""
        
        if self.pass_end_point is None:
            return None
        
        # Check if reached end of pass
        distance_to_end = math.hypot(robot_pose[0] - self.pass_end_point[0],
                                   robot_pose[1] - self.pass_end_point[1])
        
        if distance_to_end < 0.3:  # Reached end of pass
            if self.current_pass < self.total_passes - 1:
                self._transition_to_state(ZigZagState.TURN_AROUND)
                return self._turn_around_target(robot_pose)
            else:
                self._transition_to_state(ZigZagState.COMPLETED)
                return None
        
        # Continue straight pass
        self._mark_coverage(robot_pose)
        return self.pass_end_point
    
    def _turn_around_target(self, robot_pose: Tuple[float, float, float]) -> Optional[Tuple[float, float]]:
        """Target cho turn around giữa các passes"""
        
        # Move to start of next pass
        self.current_pass += 1
        self.pass_direction *= -1  # Reverse direction
        
        if self.current_pass >= self.total_passes:
            self._transition_to_state(ZigZagState.COMPLETED)
            return None
        
        self._start_new_pass()
        self._transition_to_state(ZigZagState.STRAIGHT_PASS)
        
        return self.pass_start_point
    
    def _repositioning_target(self, robot_pose: Tuple[float, float, float]) -> Optional[Tuple[float, float]]:
        """Target cho repositioning (nếu cần)"""
        
        # Simple repositioning to start of current pass
        if self.pass_start_point:
            distance_to_start = math.hypot(robot_pose[0] - self.pass_start_point[0],
                                         robot_pose[1] - self.pass_start_point[1])
            
            if distance_to_start < 0.2:
                self._transition_to_state(ZigZagState.STRAIGHT_PASS)
            
            return self.pass_start_point
        
        return None
    
    def _start_new_pass(self):
        """Bắt đầu pass mới"""
        if self.coverage_region is None:
            return
        
        # Calculate x position for this pass
        pass_x = (self.coverage_region['min_x'] + 
                 self.current_pass * self.pass_width + 
                 self.pass_width / 2)
        
        # Ensure within bounds
        pass_x = max(self.coverage_region['min_x'], 
                    min(pass_x, self.coverage_region['max_x']))
        
        if self.pass_direction == 1:
            # Forward pass: bottom to top
            self.pass_start_point = (pass_x, self.coverage_region['min_y'])
            self.pass_end_point = (pass_x, self.coverage_region['max_y'])
        else:
            # Backward pass: top to bottom
            self.pass_start_point = (pass_x, self.coverage_region['max_y'])
            self.pass_end_point = (pass_x, self.coverage_region['min_y'])
        
        print(f"🔄 [ZIGZAG] Starting pass {self.current_pass + 1}/{self.total_passes}")
        print(f"🎯 [ZIGZAG] Pass route: ({self.pass_start_point[0]:.2f}, {self.pass_start_point[1]:.2f}) → " +
              f"({self.pass_end_point[0]:.2f}, {self.pass_end_point[1]:.2f})")
    
    def _mark_coverage(self, robot_pose: Tuple[float, float, float]):
        """Đánh dấu vùng đã được cover"""
        if self.coverage_region is None:
            return
        
        grid_x = int((robot_pose[0] - self.coverage_region['origin_x']) / 
                    self.coverage_region['grid_resolution'])
        grid_y = int((robot_pose[1] - self.coverage_region['origin_y']) / 
                    self.coverage_region['grid_resolution'])
        
        # Mark coverage area around robot
        coverage_radius = int(self.robot_radius / self.coverage_region['grid_resolution'])
        for dx in range(-coverage_radius, coverage_radius + 1):
            for dy in range(-coverage_radius, coverage_radius + 1):
                if dx*dx + dy*dy <= coverage_radius*coverage_radius:
                    self.covered_cells.add((grid_x + dx, grid_y + dy))
    
    def _transition_to_state(self, new_state: ZigZagState):
        """Chuyển đổi sang state mới"""
        print(f"🔄 [ZIGZAG] Transition: {self.state.value} → {new_state.value}")
        self.state = new_state
    
    def get_completion_rate(self) -> float:
        """Lấy tỷ lệ hoàn thành zig-zag coverage"""
        if self.state == ZigZagState.COMPLETED:
            return 1.0
        elif self.total_passes == 0:
            return 0.0
        else:
            # Calculate based on completed passes
            completed_passes = self.current_pass
            current_pass_progress = 0.0
            
            # Estimate current pass progress
            if self.pass_start_point and self.pass_end_point:
                pass_length = math.hypot(self.pass_end_point[0] - self.pass_start_point[0],
                                       self.pass_end_point[1] - self.pass_start_point[1])
                if pass_length > 0:
                    current_pass_progress = min(self.current_pass_progress, 1.0)
            
            total_progress = (completed_passes + current_pass_progress) / self.total_passes
            return min(total_progress, 1.0)
    
    def get_coverage_stats(self) -> Dict:
        """Lấy thống kê coverage"""
        return {
            'current_pass': self.current_pass + 1,
            'total_passes': self.total_passes,
            'covered_cells': len(self.covered_cells),
            'completion_rate': self.get_completion_rate(),
            'state': self.state.value
        } 