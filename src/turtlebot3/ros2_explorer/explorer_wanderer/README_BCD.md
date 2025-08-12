# Thuật toán Boustrophedon Cellular Decomposition (BCD) cho Robot Hút Bụi

## Tổng quan

Thuật toán Boustrophedon Cellular Decomposition (BCD) là một phương pháp hiệu quả để thực hiện đường đi bao phủ cho robot hút bụi trong môi trường có nhiều vật cản. Thuật toán này phân chia không gian làm việc thành các ô nhỏ hơn và tạo ra đường đi bao phủ tối ưu.

## Nguyên lý hoạt động

### 1. Phân ô (Cellular Decomposition)
- **Mục đích**: Chia không gian làm việc thành các ô nhỏ hơn để dễ dàng bao phủ
- **Phương pháp**: Sử dụng các đường thẳng dọc từ các đỉnh của vật cản
- **Kết quả**: Tạo ra các ô hình thang hoặc hình chữ nhật

### 2. Đồ thị kề cận (Adjacency Graph)
- **Mỗi ô** được biểu diễn như một **nút (node)** trong đồ thị
- **Các cạnh (edge)** nối các nút của các ô kề cận nhau
- **Mục đích**: Xác định thứ tự thăm các ô

### 3. Đường đi bao phủ (Coverage Path)
- **Trong mỗi ô**: Robot di chuyển theo mô hình "boustrophedon" (qua lại)
- **Khoảng cách**: Giữa các đường thẳng bằng bán kính robot
- **An toàn**: Cách tường ít nhất 5cm

## Cấu trúc code

### 1. `bcd_coverage.py`
```python
class BCDCoverage(Node):
    """
    Thuật toán Boustrophedon Cellular Decomposition cho robot hút bụi
    """
```

**Chức năng chính:**
- Nhận bản đồ occupancy grid
- Thực hiện phân ô BCD
- Tạo đường đi bao phủ
- Publish visualization

**Các phương thức chính:**
- `perform_bcd_decomposition()`: Thực hiện phân ô
- `find_obstacle_vertices()`: Tìm đỉnh vật cản
- `generate_coverage_path()`: Tạo đường đi bao phủ
- `publish_cells_visualization()`: Hiển thị các ô

### 2. `bcd_controller.py`
```python
class BCDController(Node):
    """
    Controller cho robot thực hiện đường đi bao phủ BCD
    """
```

**Chức năng chính:**
- Nhận đường đi bao phủ từ BCD algorithm
- Điều khiển robot di chuyển theo đường đi
- Xử lý vật cản và dừng khẩn cấp
- Theo dõi tiến độ bao phủ

## Cách sử dụng

### 1. Chạy hệ thống BCD Coverage
```bash
# Chạy launch file
ros2 launch explorer_bringup bcd_coverage_launch.py

# Hoặc chạy từng node riêng lẻ
ros2 run explorer_wanderer bcd_coverage
ros2 run explorer_wanderer bcd_controller
```

### 2. Theo dõi visualization
```bash
# Xem các ô BCD
ros2 topic echo /bcd_cells

# Xem đường đi bao phủ
ros2 topic echo /coverage_path
```

### 3. Kiểm tra trạng thái
```bash
# Xem danh sách nodes
ros2 node list

# Xem topics
ros2 topic list

# Xem services
ros2 service list
```

## Tham số cấu hình

### Trong `bcd_coverage.py`:
```python
self.robot_radius = 0.1  # Bán kính robot (m)
self.coverage_spacing = 0.15  # Khoảng cách giữa các đường bao phủ (m)
self.wall_distance = 0.05  # Khoảng cách với tường (m)
```

### Trong `bcd_controller.py`:
```python
self.safety_distance = 0.3  # Khoảng cách an toàn với vật cản
self.linear_speed = 0.2  # Tốc độ di chuyển
self.angular_speed = 0.5  # Tốc độ quay
```

## Ưu điểm của thuật toán BCD

### 1. Hiệu quả cao
- **Bao phủ hoàn toàn**: Đảm bảo tất cả khu vực được bao phủ
- **Tối ưu đường đi**: Giảm thiểu thời gian di chuyển
- **Tránh lặp lại**: Không đi qua cùng một khu vực nhiều lần

### 2. Thích ứng tốt
- **Xử lý vật cản phức tạp**: Hoạt động tốt với nhiều hình dạng vật cản
- **Môi trường động**: Có thể cập nhật khi có vật cản mới
- **Kích thước linh hoạt**: Hoạt động với các kích thước môi trường khác nhau

### 3. An toàn
- **Khoảng cách an toàn**: Tự động duy trì khoảng cách với tường
- **Phát hiện vật cản**: Dừng khi phát hiện vật cản
- **Dừng khẩn cấp**: Xử lý tình huống khẩn cấp

## So sánh với các phương pháp khác

| Phương pháp | Ưu điểm | Nhược điểm |
|-------------|---------|------------|
| **BCD** | Bao phủ hoàn toàn, tối ưu đường đi | Phức tạp tính toán |
| **Random Walk** | Đơn giản | Không hiệu quả, có thể bỏ sót |
| **Wall Following** | Đơn giản | Chỉ hiệu quả với môi trường đơn giản |
| **Grid-based** | Dễ hiểu | Không tối ưu với vật cản phức tạp |

## Troubleshooting

### 1. Robot không di chuyển
```bash
# Kiểm tra navigation stack
ros2 topic echo /cmd_vel

# Kiểm tra action server
ros2 action list
```

### 2. Không tạo được đường đi bao phủ
```bash
# Kiểm tra bản đồ
ros2 topic echo /map

# Kiểm tra BCD coverage
ros2 topic echo /bcd_cells
```

### 3. Robot bị kẹt
```bash
# Dừng khẩn cấp
ros2 service call /emergency_stop std_srvs/srv/Trigger

# Reset navigation
ros2 service call /reset_navigation std_srvs/srv/Trigger
```

## Tài liệu tham khảo

1. Choset, H., et al. "Principles of Robot Motion: Theory, Algorithms, and Implementations"
2. Galceran, E., & Carreras, M. "A survey on coverage path planning for robotics"
3. Xu, L., et al. "Boustrophedon cellular decomposition: A theoretical framework for path planning"

## Liên hệ

Nếu có vấn đề hoặc câu hỏi về thuật toán BCD, vui lòng tạo issue trên GitHub repository. 