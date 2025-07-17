#!/usr/bin/env python3
"""
Demo file cho Smart Vacuum Cleaning Algorithm
Minh họa cách sử dụng thuật toán 3 giai đoạn: MAPPING → BOUNDARY_WALK → ZIGZAG_COVERAGE
"""

import numpy as np
import matplotlib.pyplot as plt
import time
from smart_cleaning_algorithm import SmartCleaningAlgorithm, CleaningPhase

def create_demo_environment():
    """Tạo môi trường demo cho robot hút bụi"""
    # Tạo occupancy grid 20x20 meters (400x400 cells với resolution 0.05)
    width, height = 400, 400
    occupancy_grid = np.zeros((height, width), dtype=np.int8)
    
    # Thêm tường bao quanh
    occupancy_grid[0, :] = 100  # Top wall
    occupancy_grid[-1, :] = 100  # Bottom wall
    occupancy_grid[:, 0] = 100  # Left wall
    occupancy_grid[:, -1] = 100  # Right wall
    
    # Thêm một số chướng ngại vật bên trong
    # Rectangular obstacle
    occupancy_grid[100:150, 150:250] = 100
    
    # L-shaped obstacle
    occupancy_grid[200:300, 100:120] = 100
    occupancy_grid[280:300, 100:200] = 100
    
    # Circular obstacle
    center_x, center_y = 300, 300
    radius = 30
    for i in range(height):
        for j in range(width):
            if (i - center_x)**2 + (j - center_y)**2 <= radius**2:
                occupancy_grid[i, j] = 100
    
    # Thêm unknown areas để mô phỏng quá trình mapping
    occupancy_grid[50:350, 50:350][occupancy_grid[50:350, 50:350] == 0] = -1
    
    # Chỉ giữ một phần đã được explore
    occupancy_grid[180:220, 180:220] = 0  # Small explored area
    
    return occupancy_grid

def simulate_laser_scan(robot_pose, occupancy_grid, max_range=5.0, resolution=0.05):
    """Mô phỏng laser scan từ robot position"""
    robot_x, robot_y, robot_yaw = robot_pose
    
    # Convert to grid coordinates
    robot_col = int(robot_x / resolution)
    robot_row = int(robot_y / resolution)
    
    ranges = []
    num_beams = 360
    
    for i in range(num_beams):
        angle = (i * 2 * np.pi / num_beams) + robot_yaw
        
        # Ray casting
        max_cells = int(max_range / resolution)
        for step in range(1, max_cells):
            ray_x = robot_col + step * np.cos(angle)
            ray_y = robot_row + step * np.sin(angle)
            
            ray_col = int(ray_x)
            ray_row = int(ray_y)
            
            # Check bounds
            if (ray_row < 0 or ray_row >= occupancy_grid.shape[0] or
                ray_col < 0 or ray_col >= occupancy_grid.shape[1]):
                ranges.append(max_range)
                break
            
            # Check obstacle
            if occupancy_grid[ray_row, ray_col] == 100:
                distance = step * resolution
                ranges.append(distance)
                break
        else:
            ranges.append(max_range)
    
    return ranges

def visualize_cleaning_progress(occupancy_grid, robot_pose, cleaned_area, 
                               current_target, algorithm_info):
    """Visualize tiến độ dọn dẹp"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    # Plot 1: Environment and robot
    ax1.imshow(occupancy_grid, cmap='gray', origin='lower')
    
    # Plot robot position
    robot_col = int(robot_pose[0] / 0.05)
    robot_row = int(robot_pose[1] / 0.05)
    ax1.plot(robot_col, robot_row, 'ro', markersize=8, label='Robot')
    
    # Plot target
    if current_target:
        target_col = int(current_target[0] / 0.05)
        target_row = int(current_target[1] / 0.05)
        ax1.plot(target_col, target_row, 'g*', markersize=12, label='Target')
    
    # Plot cleaned area
    if cleaned_area:
        cleaned_rows, cleaned_cols = zip(*cleaned_area)
        ax1.scatter(cleaned_cols, cleaned_rows, c='lightblue', s=1, alpha=0.5, label='Cleaned')
    
    ax1.set_title(f"Smart Vacuum Cleaning - Phase: {algorithm_info['current_phase'].upper()}")
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Statistics
    ax2.axis('off')
    stats_text = f"""
🤖 SMART VACUUM CLEANING STATUS

📍 Current Phase: {algorithm_info['current_phase'].upper()}

📊 Progress:
  🗺️  Mapping: {algorithm_info['mapping_confidence']:.1%}
  🚶 Boundary: {algorithm_info['boundary_completion']:.1%}  
  🔄 Coverage: {algorithm_info['coverage_completion']:.1%}

📈 Statistics:
  🧹 Cleaned Cells: {algorithm_info['cleaned_cells']}
  🧱 Boundary Segments: {algorithm_info['boundary_segments']}
  📋 Zig-zag Regions: {algorithm_info['zigzag_regions']}

🎯 Robot Position: ({robot_pose[0]:.2f}, {robot_pose[1]:.2f})
"""
    
    if algorithm_info['current_phase'] == 'boundary':
        if 'boundary_state' in algorithm_info:
            stats_text += f"\n🚶 Boundary State: {algorithm_info['boundary_state']}"
        if 'boundary_followed_points' in algorithm_info:
            stats_text += f"\n📏 Followed Points: {algorithm_info['boundary_followed_points']}"
    
    elif algorithm_info['current_phase'] == 'zigzag':
        if 'zigzag_state' in algorithm_info:
            stats_text += f"\n🔄 Zig-zag State: {algorithm_info['zigzag_state']}"
        if 'current_pass' in algorithm_info and 'total_passes' in algorithm_info:
            stats_text += f"\n📋 Pass: {algorithm_info['current_pass']}/{algorithm_info['total_passes']}"
    
    ax2.text(0.1, 0.9, stats_text, transform=ax2.transAxes, fontsize=11, 
             verticalalignment='top', fontfamily='monospace')
    
    plt.tight_layout()
    plt.pause(0.5)
    plt.clf()

def main():
    """Main demo function"""
    print("🧹 Starting Smart Vacuum Cleaning Demo")
    print("=" * 50)
    
    # Tạo environment
    occupancy_grid = create_demo_environment()
    print(f"📍 Created demo environment: {occupancy_grid.shape}")
    
    # Khởi tạo smart cleaning algorithm
    smart_cleaner = SmartCleaningAlgorithm(
        robot_radius=0.2,
        grid_resolution=0.05
    )
    
    # Robot starting position (center of explored area)
    robot_pose = [9.0, 9.0, 0.0]  # x, y, yaw
    
    print(f"🤖 Robot starting at: ({robot_pose[0]:.2f}, {robot_pose[1]:.2f})")
    
    # Simulation parameters
    max_steps = 1000
    step_size = 0.2  # meters per step
    
    plt.ion()  # Interactive plotting
    
    for step in range(max_steps):
        print(f"\n--- Step {step + 1} ---")
        
        # Simulate laser scan
        scan_ranges = simulate_laser_scan(tuple(robot_pose), occupancy_grid)
        
        # Update algorithm phase
        current_phase = smart_cleaner.update_phase(
            occupancy_grid, tuple(robot_pose), scan_ranges
        )
        
        # Check completion
        if current_phase == CleaningPhase.COMPLETED:
            print("🎉 Smart vacuum cleaning COMPLETED!")
            break
        
        # Get next target
        target = smart_cleaner.get_next_target(occupancy_grid, tuple(robot_pose))
        
        if target is None:
            print("⚠️ No target available, completing...")
            break
        
        print(f"🎯 Target: ({target[0]:.2f}, {target[1]:.2f})")
        
        # Simulate movement towards target
        dx = target[0] - robot_pose[0]
        dy = target[1] - robot_pose[1]
        distance = np.sqrt(dx**2 + dy**2)
        
        if distance > step_size:
            # Move towards target
            robot_pose[0] += (dx / distance) * step_size
            robot_pose[1] += (dy / distance) * step_size
            # Update heading
            robot_pose[2] = np.arctan2(dy, dx)
        else:
            # Reached target
            robot_pose[0] = target[0]
            robot_pose[1] = target[1]
            smart_cleaner.mark_area_cleaned(tuple(robot_pose))
            print("✅ Target reached and area cleaned")
        
        # Visualize progress
        if step % 5 == 0:  # Update visualization every 5 steps
            algorithm_info = smart_cleaner.get_phase_info()
            visualize_cleaning_progress(
                occupancy_grid, tuple(robot_pose), 
                smart_cleaner.cleaned_area, target, algorithm_info
            )
        
        # Log progress
        if step % 20 == 0:
            algorithm_info = smart_cleaner.get_phase_info()
            print(f"📊 Progress - Phase: {algorithm_info['current_phase']}")
            print(f"   Mapping: {algorithm_info['mapping_confidence']:.1%}")
            print(f"   Boundary: {algorithm_info['boundary_completion']:.1%}")
            print(f"   Coverage: {algorithm_info['coverage_completion']:.1%}")
    
    plt.ioff()
    
    # Final statistics
    final_info = smart_cleaner.get_phase_info()
    print("\n" + "=" * 50)
    print("🏁 CLEANING SIMULATION COMPLETED")
    print("=" * 50)
    print(f"📊 Final Statistics:")
    print(f"   🧹 Total cleaned cells: {final_info['cleaned_cells']}")
    print(f"   📈 Final coverage: {final_info['coverage_completion']:.1%}")
    print(f"   ⏱️  Total steps: {step + 1}")
    print(f"   🎯 Final phase: {final_info['current_phase'].upper()}")

if __name__ == "__main__":
    main() 