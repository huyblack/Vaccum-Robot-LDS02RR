import numpy as np
import math
from enum import Enum
import cv2
from typing import List, Tuple, Optional
import heapq

# Import các thuật toán chi tiết
from vacuum_algorithms import BoundaryWalkAlgorithm, ZigZagCoverageAlgorithm, BoundaryWalkState, ZigZagState

class CleaningPhase(Enum):
    """Các giai đoạn của thuật toán dọn dẹp thông minh"""
    MAPPING = "mapping"           # Giai đoạn quét và mapping môi trường
    BOUNDARY_WALK = "boundary"    # Giai đoạn đi dọc theo các cạnh/biên  
    ZIGZAG_COVERAGE = "zigzag"    # Giai đoạn cover vùng bên trong bằng zig-zag
    COMPLETED = "completed"       # Hoàn thành dọn dẹp

class SmartCleaningAlgorithm:
    """
    Thuật toán dọn dẹp thông minh cho robot hút bụi
    Kết hợp 3 giai đoạn: SLAM Mapping → Boundary Walk → Zig-zag Coverage
    """
    
    def __init__(self, robot_radius=0.2, grid_resolution=0.05):
        self.robot_radius = robot_radius
        self.grid_resolution = grid_resolution
        self.current_phase = CleaningPhase.MAPPING
        
        # Tracking coverage
        self.cleaned_area = set()
        self.boundary_segments = []
        self.zigzag_regions = []
        
        # State management
        self.mapping_confidence = 0.0
        self.boundary_completion = 0.0
        self.coverage_completion = 0.0
        
        # Algorithm parameters
        self.min_mapping_confidence = 0.85  # Ngưỡng để chuyển sang boundary walk
        self.wall_follow_distance = 0.3     # Khoảng cách giữ với tường
        self.zigzag_width = 0.4             # Độ rộng mỗi pass zig-zag
        
        # Initialize specialized algorithms
        self.boundary_walker = BoundaryWalkAlgorithm(
            wall_distance=self.wall_follow_distance,
            robot_radius=robot_radius
        )
        
        self.zigzag_coverage = ZigZagCoverageAlgorithm(
            pass_width=self.zigzag_width,
            robot_radius=robot_radius
        )
        
        # Additional state tracking
        self.scan_data = None
        self.last_robot_pose = None
        
        print("🤖 [INFO] Smart Cleaning Algorithm initialized!")
        print("📍 Phase 1: SLAM Mapping active")
    
    def update_phase(self, occupancy_grid: np.ndarray, robot_pose: Tuple[float, float, float], 
                    scan_ranges: List[float] = None) -> CleaningPhase:
        """
        Cập nhật giai đoạn dựa trên tiến độ và điều kiện
        """
        self.scan_data = scan_ranges
        self.last_robot_pose = robot_pose
        
        if self.current_phase == CleaningPhase.MAPPING:
            # Tính toán độ tin cậy của map
            self.mapping_confidence = self._calculate_mapping_confidence(occupancy_grid)
            
            if self.mapping_confidence >= self.min_mapping_confidence:
                print(f"🗺️ [INFO] Mapping completed with {self.mapping_confidence:.2%} confidence")
                print("🚶 [INFO] Switching to Phase 2: Boundary Walk")
                self.current_phase = CleaningPhase.BOUNDARY_WALK
                self._detect_boundaries(occupancy_grid)
        
        elif self.current_phase == CleaningPhase.BOUNDARY_WALK:
            # Cập nhật boundary completion từ boundary walker
            self.boundary_completion = self.boundary_walker.get_completion_rate()
            
            if (self.boundary_completion >= 0.95 or 
                self.boundary_walker.state == BoundaryWalkState.COMPLETED):
                print(f"🚶 [INFO] Boundary walk completed: {self.boundary_completion:.2%}")
                print("🔄 [INFO] Switching to Phase 3: Zig-zag Coverage")
                self.current_phase = CleaningPhase.ZIGZAG_COVERAGE
                self._plan_zigzag_regions(occupancy_grid)
        
        elif self.current_phase == CleaningPhase.ZIGZAG_COVERAGE:
            # Cập nhật zig-zag completion
            self.coverage_completion = self.zigzag_coverage.get_completion_rate()
            
            if (self.coverage_completion >= 0.98 or 
                self.zigzag_coverage.state == ZigZagState.COMPLETED):
                print(f"🔄 [INFO] Zig-zag coverage completed: {self.coverage_completion:.2%}")
                print("✅ [INFO] Smart cleaning COMPLETED!")
                self.current_phase = CleaningPhase.COMPLETED
        
        return self.current_phase
    
    def get_next_target(self, occupancy_grid: np.ndarray, 
                       robot_pose: Tuple[float, float, float]) -> Optional[Tuple[float, float]]:
        """
        Lấy target tiếp theo dựa trên giai đoạn hiện tại
        """
        if self.current_phase == CleaningPhase.MAPPING:
            return self._get_mapping_target(occupancy_grid, robot_pose)
        
        elif self.current_phase == CleaningPhase.BOUNDARY_WALK:
            return self._get_boundary_target(occupancy_grid, robot_pose)
        
        elif self.current_phase == CleaningPhase.ZIGZAG_COVERAGE:
            return self._get_zigzag_target(occupancy_grid, robot_pose)
        
        else:  # COMPLETED
            return None
    
    def _calculate_mapping_confidence(self, occupancy_grid: np.ndarray) -> float:
        """
        Tính độ tin cậy của map dựa trên tỷ lệ vùng đã được khám phá
        """
        total_cells = occupancy_grid.size
        unknown_cells = np.sum(occupancy_grid == -1)  # -1 = unknown
        explored_ratio = (total_cells - unknown_cells) / total_cells
        return explored_ratio
    
    def _detect_boundaries(self, occupancy_grid: np.ndarray):
        """
        Phát hiện các boundaries/edges trong môi trường sử dụng edge detection
        """
        print("🔍 [INFO] Detecting boundaries...")
        
        # Chuyển đổi occupancy grid thành binary image
        binary_map = np.zeros_like(occupancy_grid, dtype=np.uint8)
        binary_map[occupancy_grid == 100] = 255  # Walls = white
        binary_map[occupancy_grid == 0] = 0      # Free space = black
        binary_map[occupancy_grid == -1] = 128   # Unknown = gray
        
        # Sử dụng Canny edge detection
        edges = cv2.Canny(binary_map, 50, 150)
        
        # Tìm contours (boundary segments)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        self.boundary_segments = []
        for contour in contours:
            # Convert từ pixel coordinates sang world coordinates
            if len(contour) > 10:  # Chỉ giữ contours đủ dài
                world_contour = []
                for point in contour:
                    x_world = point[0][1] * self.grid_resolution  
                    y_world = point[0][0] * self.grid_resolution
                    world_contour.append((x_world, y_world))
                self.boundary_segments.append(world_contour)
        
        print(f"🔍 [INFO] Found {len(self.boundary_segments)} boundary segments")
    
    def _plan_zigzag_regions(self, occupancy_grid: np.ndarray):
        """
        Lập kế hoạch các vùng zig-zag cho coverage sau khi boundary walk
        """
        print("📋 [INFO] Planning zig-zag regions...")
        
        # Sử dụng zigzag algorithm để initialize coverage region
        self.zigzag_coverage.initialize_coverage_region(
            occupancy_grid, self.grid_resolution, 0.0, 0.0
        )
        
        print(f"📋 [INFO] Zig-zag coverage region initialized")
    
    def _get_mapping_target(self, occupancy_grid: np.ndarray, 
                           robot_pose: Tuple[float, float, float]) -> Optional[Tuple[float, float]]:
        """
        Lấy target cho giai đoạn mapping (sử dụng frontier-based exploration)
        """
        # Sử dụng frontier detection đơn giản để tránh circular import
        return self._simple_frontier_detection(occupancy_grid, robot_pose)
    
    def _get_boundary_target(self, occupancy_grid: np.ndarray,
                            robot_pose: Tuple[float, float, float]) -> Optional[Tuple[float, float]]:
        """
        Lấy target cho giai đoạn boundary walk sử dụng BoundaryWalkAlgorithm
        """
        if self.scan_data is None:
            print("⚠️ [WARNING] No scan data available for boundary walk")
            return None
        
        # Sử dụng boundary walker để lấy target
        target = self.boundary_walker.get_next_target(
            occupancy_grid, robot_pose, self.scan_data,
            self.grid_resolution, 0.0, 0.0  # Assume origin at (0,0)
        )
        
        # Update cleaned area từ boundary walker
        if target and self.boundary_walker.followed_boundaries:
            self.cleaned_area.update(self.boundary_walker.followed_boundaries)
        
        return target
    
    def _get_zigzag_target(self, occupancy_grid: np.ndarray,
                          robot_pose: Tuple[float, float, float]) -> Optional[Tuple[float, float]]:
        """
        Lấy target cho giai đoạn zig-zag coverage sử dụng ZigZagCoverageAlgorithm
        """
        # Sử dụng zig-zag algorithm để lấy target
        target = self.zigzag_coverage.get_next_target(robot_pose, occupancy_grid)
        
        # Update cleaned area từ zig-zag coverage
        if target and self.zigzag_coverage.covered_cells:
            self.cleaned_area.update(self.zigzag_coverage.covered_cells)
        
        return target

    def _simple_frontier_detection(self, occupancy_grid: np.ndarray,
                                 robot_pose: Tuple[float, float, float]) -> Optional[Tuple[float, float]]:
        """
        Frontier detection đơn giản để tránh circular import
        """
        print("🔍 [INFO] Running simple frontier detection...")
        
        height, width = occupancy_grid.shape
        frontiers = []
        
        # Tìm frontier cells (free cells adjacent to unknown cells)
        for i in range(1, height-1):
            for j in range(1, width-1):
                if occupancy_grid[i, j] == 0:  # Free cell
                    # Kiểm tra các neighbors
                    has_unknown_neighbor = False
                    for di in [-1, 0, 1]:
                        for dj in [-1, 0, 1]:
                            ni, nj = i + di, j + dj
                            if 0 <= ni < height and 0 <= nj < width:
                                if occupancy_grid[ni, nj] == -1:  # Unknown
                                    has_unknown_neighbor = True
                                    break
                        if has_unknown_neighbor:
                            break
                    
                    if has_unknown_neighbor:
                        # Convert to world coordinates
                        world_x = j * self.grid_resolution
                        world_y = i * self.grid_resolution
                        frontiers.append((world_x, world_y))
        
        if not frontiers:
            print("⚠️ [WARNING] No frontiers found, using fallback")
            return self._fallback_exploration_target(occupancy_grid, robot_pose)
        
        # Chọn frontier gần nhất
        robot_x, robot_y = robot_pose[0], robot_pose[1]
        min_distance = float('inf')
        best_frontier = None
        
        for fx, fy in frontiers:
            distance = math.hypot(robot_x - fx, robot_y - fy)
            if distance < min_distance and distance > 0.5:  # Ít nhất 0.5m từ robot
                min_distance = distance
                best_frontier = (fx, fy)
        
        if best_frontier:
            print(f"🎯 [INFO] Found frontier target at ({best_frontier[0]:.2f}, {best_frontier[1]:.2f})")
        else:
            print("⚠️ [WARNING] No suitable frontiers found, using fallback")
            return self._fallback_exploration_target(occupancy_grid, robot_pose)
        
        return best_frontier

    def _fallback_exploration_target(self, occupancy_grid: np.ndarray,
                                   robot_pose: Tuple[float, float, float]) -> Optional[Tuple[float, float]]:
        """
        Fallback exploration khi không có frontier detection
        """
        # Simple exploration: tìm unknown area gần nhất
        unknown_positions = np.where(occupancy_grid == -1)
        if len(unknown_positions[0]) > 0:
            robot_col = int(robot_pose[0] / self.grid_resolution)
            robot_row = int(robot_pose[1] / self.grid_resolution)
            
            min_distance = float('inf')
            best_target = None
            
            for i in range(0, len(unknown_positions[0]), 10):  # Sample every 10th point
                target_row, target_col = unknown_positions[0][i], unknown_positions[1][i]
                distance = math.hypot(robot_row - target_row, robot_col - target_col)
                
                if distance < min_distance:
                    min_distance = distance
                    target_x = target_col * self.grid_resolution
                    target_y = target_row * self.grid_resolution
                    best_target = (target_x, target_y)
            
            return best_target
        
        return None
    
    def _calculate_boundary_completion(self) -> float:
        """Tính tỷ lệ hoàn thành boundary walk"""
        return self.boundary_walker.get_completion_rate()
    
    def _calculate_coverage_completion(self, occupancy_grid: np.ndarray) -> float:
        """Tính tỷ lệ hoàn thành coverage"""
        return self.zigzag_coverage.get_completion_rate()
    
    def get_phase_info(self) -> dict:
        """Lấy thông tin chi tiết về giai đoạn hiện tại"""
        info = {
            'current_phase': self.current_phase.value,
            'mapping_confidence': self.mapping_confidence,
            'boundary_completion': self.boundary_completion,
            'coverage_completion': self.coverage_completion,
            'cleaned_cells': len(self.cleaned_area),
            'boundary_segments': len(self.boundary_segments),
            'zigzag_regions': 0  # Will be updated below
        }
        
        # Add detailed info based on current phase
        if self.current_phase == CleaningPhase.BOUNDARY_WALK:
            info['boundary_state'] = self.boundary_walker.state.value
            info['boundary_followed_points'] = len(self.boundary_walker.followed_boundaries)
        
        elif self.current_phase == CleaningPhase.ZIGZAG_COVERAGE:
            coverage_stats = self.zigzag_coverage.get_coverage_stats()
            info.update({
                'zigzag_state': coverage_stats['state'],
                'current_pass': coverage_stats['current_pass'],
                'total_passes': coverage_stats['total_passes'],
                'zigzag_regions': coverage_stats['total_passes']
            })
        
        return info
    
    def mark_area_cleaned(self, robot_pose: Tuple[float, float, float]):
        """Đánh dấu vùng xung quanh robot đã được dọn dẹp"""
        grid_x = int(robot_pose[0] / self.grid_resolution)
        grid_y = int(robot_pose[1] / self.grid_resolution)
        
        # Mark một vùng nhỏ xung quanh robot
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                if dx*dx + dy*dy <= 4:  # Circular area
                    self.cleaned_area.add((grid_x + dx, grid_y + dy)) 