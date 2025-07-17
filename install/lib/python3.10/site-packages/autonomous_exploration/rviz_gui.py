#!/usr/bin/env python3

import sys
import os
import subprocess
import threading
import time
import signal
from datetime import datetime
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                             QWidget, QPushButton, QLabel, QTextEdit, QMessageBox,
                             QProgressBar, QFrame, QGridLayout, QSplitter)
from PyQt5.QtCore import QThread, pyqtSignal, QTimer, Qt, QProcess
from PyQt5.QtGui import QFont, QPalette, QColor, QPixmap, QIcon
import rclpy
from rclpy.node import Node

class SLAMController(QThread):
    """Thread để điều khiển quá trình SLAM"""
    output_received = pyqtSignal(str)
    exploration_completed = pyqtSignal()
    slam_started = pyqtSignal()
    slam_stopped = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        self.process = None
        self.running = False
        
    def start_slam(self):
        """Bắt đầu SLAM"""
        if self.running:
            return False
            
        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            control_path = os.path.join(script_dir, 'control.py')
            
            if not os.path.exists(control_path):
                self.output_received.emit("❌ Không tìm thấy file control.py")
                return False
            
            # Khởi động control.py với Python3
            self.process = subprocess.Popen(
                ['python3', control_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1,
                preexec_fn=os.setsid  # Tạo process group riêng
            )
            
            self.running = True
            self.start()
            self.slam_started.emit()
            return True
            
        except Exception as e:
            self.output_received.emit(f"❌ Lỗi khởi động SLAM: {str(e)}")
            return False
    
    def stop_slam(self):
        """Dừng SLAM"""
        if self.process and self.running:
            try:
                # Dừng toàn bộ process group
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                self.process.wait(timeout=5)
            except:
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                except:
                    pass
            
            self.running = False
            self.slam_stopped.emit()
    
    def run(self):
        """Theo dõi output của SLAM"""
        if not self.process:
            return
            
        while self.running and self.process.poll() is None:
            try:
                line = self.process.stdout.readline()
                if line:
                    line = line.strip()
                    self.output_received.emit(line)
                    
                    # Kiểm tra thông báo hoàn thành (cả tiếng Thổ Nhĩ Kỳ và tiếng Anh)
                    if ("KESİF TAMAMLANDI" in line or 
                        "EXPLORATION COMPLETED" in line):
                        self.exploration_completed.emit()
                        
            except Exception as e:
                self.output_received.emit(f"❌ Lỗi đọc output: {str(e)}")
                break
        
        self.running = False

class RVizController(QThread):
    """Thread để điều khiển RViz"""
    rviz_started = pyqtSignal()
    rviz_stopped = pyqtSignal()
    status_changed = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.process = None
        self.running = False
        
    def start_rviz(self, config_path=None):
        """Khởi động RViz"""
        if self.running:
            return False
            
        try:
            cmd = ['rviz2']
            if config_path and os.path.exists(config_path):
                cmd.extend(['-d', config_path])
            
            # Vô hiệu hóa Qt platform integration để tránh xung đột
            env = os.environ.copy()
            env['QT_QPA_PLATFORM_PLUGIN_PATH'] = ''
            
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=env,
                preexec_fn=os.setsid
            )
            
            self.running = True
            self.start()
            self.rviz_started.emit()
            return True
            
        except Exception as e:
            self.status_changed.emit(f"❌ Lỗi khởi động RViz: {str(e)}")
            return False
    
    def stop_rviz(self):
        """Dừng RViz"""
        if self.process and self.running:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                self.process.wait(timeout=5)
            except:
                try:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
                except:
                    pass
            
            self.running = False
            self.rviz_stopped.emit()
    
    def run(self):
        """Theo dõi trạng thái RViz"""
        while self.running and self.process and self.process.poll() is None:
            time.sleep(1)
        
        self.running = False
        self.rviz_stopped.emit()

class RVizGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Khởi tạo controllers
        self.slam_controller = SLAMController()
        self.rviz_controller = RVizController()
        
        # Thiết lập UI
        self.setup_ui()
        self.setup_connections()
        
        # Timer để cập nhật trạng thái
        self.status_timer = QTimer()
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(2000)
        
        # Đường dẫn config RViz
        self.config_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), 
            'config', 'slam_config.rviz'
        )
        
    def setup_ui(self):
        """Thiết lập giao diện người dùng"""
        self.setWindowTitle("🤖 ROS2 SLAM Controller & RViz")
        self.setGeometry(100, 100, 900, 700)
        
        # Widget chính
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Layout chính với splitter
        main_layout = QHBoxLayout(central_widget)
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)
        
        # Panel điều khiển bên trái
        control_panel = self.create_control_panel()
        splitter.addWidget(control_panel)
        
        # Panel log bên phải
        log_panel = self.create_log_panel()
        splitter.addWidget(log_panel)
        
        # Thiết lập tỷ lệ splitter
        splitter.setSizes([400, 500])
        
        # Áp dụng style
        self.apply_styles()
    
    def create_control_panel(self):
        """Tạo panel điều khiển"""
        panel = QFrame()
        layout = QVBoxLayout(panel)
        
        # Tiêu đề
        title = QLabel("🎮 BẢN ĐIỀU KHIỂN SLAM")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Trạng thái hệ thống
        status_frame = QFrame()
        status_frame.setFrameStyle(QFrame.Box)
        status_layout = QVBoxLayout(status_frame)
        
        status_title = QLabel("📊 Trạng thái hệ thống")
        status_title.setFont(QFont("Arial", 11, QFont.Bold))
        status_layout.addWidget(status_title)
        
        self.slam_status = QLabel("🔴 SLAM: Chưa khởi động")
        self.rviz_status = QLabel("🔴 RViz: Chưa khởi động")
        self.ros_status = QLabel("🔴 ROS2: Kiểm tra...")
        
        status_layout.addWidget(self.slam_status)
        status_layout.addWidget(self.rviz_status)
        status_layout.addWidget(self.ros_status)
        layout.addWidget(status_frame)
        
        # Điều khiển SLAM
        slam_frame = QFrame()
        slam_frame.setFrameStyle(QFrame.Box)
        slam_layout = QVBoxLayout(slam_frame)
        
        slam_title = QLabel("🗺️ Điều khiển SLAM")
        slam_title.setFont(QFont("Arial", 11, QFont.Bold))
        slam_layout.addWidget(slam_title)
        
        # Buttons SLAM
        slam_buttons = QHBoxLayout()
        
        self.start_slam_btn = QPushButton("▶️ BẮT ĐẦU")
        self.start_slam_btn.setMinimumHeight(40)
        self.start_slam_btn.setStyleSheet("""
            QPushButton {
                background-color: #28a745;
                color: white;
                font-weight: bold;
                border-radius: 8px;
                padding: 8px;
            }
            QPushButton:hover {
                background-color: #218838;
            }
            QPushButton:disabled {
                background-color: #6c757d;
            }
        """)
        
        self.stop_slam_btn = QPushButton("⏹️ DỪNG")
        self.stop_slam_btn.setMinimumHeight(40)
        self.stop_slam_btn.setEnabled(False)
        self.stop_slam_btn.setStyleSheet("""
            QPushButton {
                background-color: #dc3545;
                color: white;
                font-weight: bold;
                border-radius: 8px;
                padding: 8px;
            }
            QPushButton:hover {
                background-color: #c82333;
            }
            QPushButton:disabled {
                background-color: #6c757d;
            }
        """)
        
        slam_buttons.addWidget(self.start_slam_btn)
        slam_buttons.addWidget(self.stop_slam_btn)
        slam_layout.addLayout(slam_buttons)
        
        # Progress bar cho SLAM
        self.slam_progress = QProgressBar()
        self.slam_progress.setVisible(False)
        slam_layout.addWidget(self.slam_progress)
        
        layout.addWidget(slam_frame)
        
        # Điều khiển RViz
        rviz_frame = QFrame()
        rviz_frame.setFrameStyle(QFrame.Box)
        rviz_layout = QVBoxLayout(rviz_frame)
        
        rviz_title = QLabel("👁️ Điều khiển RViz")
        rviz_title.setFont(QFont("Arial", 11, QFont.Bold))
        rviz_layout.addWidget(rviz_title)
        
        # Buttons RViz
        rviz_buttons = QHBoxLayout()
        
        self.start_rviz_btn = QPushButton("🚀 KHỞI ĐỘNG")
        self.start_rviz_btn.setMinimumHeight(40)
        self.start_rviz_btn.setStyleSheet("""
            QPushButton {
                background-color: #007bff;
                color: white;
                font-weight: bold;
                border-radius: 8px;
                padding: 8px;
            }
            QPushButton:hover {
                background-color: #0056b3;
            }
            QPushButton:disabled {
                background-color: #6c757d;
            }
        """)
        
        self.stop_rviz_btn = QPushButton("❌ ĐÓNG")
        self.stop_rviz_btn.setMinimumHeight(40)
        self.stop_rviz_btn.setEnabled(False)
        self.stop_rviz_btn.setStyleSheet("""
            QPushButton {
                background-color: #dc3545;
                color: white;
                font-weight: bold;
                border-radius: 8px;
                padding: 8px;
            }
            QPushButton:hover {
                background-color: #c82333;
            }
            QPushButton:disabled {
                background-color: #6c757d;
            }
        """)
        
        rviz_buttons.addWidget(self.start_rviz_btn)
        rviz_buttons.addWidget(self.stop_rviz_btn)
        rviz_layout.addLayout(rviz_buttons)
        
        layout.addWidget(rviz_frame)
        
        # Quản lý Map
        map_frame = QFrame()
        map_frame.setFrameStyle(QFrame.Box)
        map_layout = QVBoxLayout(map_frame)
        
        map_title = QLabel("💾 Quản lý Map")
        map_title.setFont(QFont("Arial", 11, QFont.Bold))
        map_layout.addWidget(map_title)
        
        # Buttons Map
        map_buttons = QHBoxLayout()
        
        self.save_map_btn = QPushButton("💾 LƯU")
        self.save_map_btn.setMinimumHeight(35)
        self.save_map_btn.setStyleSheet("""
            QPushButton {
                background-color: #28a745;
                color: white;
                font-weight: bold;
                border-radius: 5px;
                padding: 5px;
            }
            QPushButton:hover {
                background-color: #218838;
            }
        """)
        
        self.load_map_btn = QPushButton("📂 TẢI")
        self.load_map_btn.setMinimumHeight(35)
        self.load_map_btn.setStyleSheet("""
            QPushButton {
                background-color: #17a2b8;
                color: white;
                font-weight: bold;
                border-radius: 5px;
                padding: 5px;
            }
            QPushButton:hover {
                background-color: #138496;
            }
        """)
        
        map_buttons.addWidget(self.save_map_btn)
        map_buttons.addWidget(self.load_map_btn)
        map_layout.addLayout(map_buttons)
        
        layout.addWidget(map_frame)
        
        # Nút thoát
        layout.addStretch()
        
        self.exit_btn = QPushButton("🚪 THOÁT")
        self.exit_btn.setMinimumHeight(45)
        self.exit_btn.setStyleSheet("""
            QPushButton {
                background-color: #6c757d;
                color: white;
                font-weight: bold;
                border-radius: 8px;
                padding: 10px;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #545b62;
            }
        """)
        layout.addWidget(self.exit_btn)
        
        return panel
    
    def create_log_panel(self):
        """Tạo panel hiển thị log"""
        panel = QFrame()
        layout = QVBoxLayout(panel)
        
        # Tiêu đề log
        log_title = QLabel("📋 NHẬT KÝ HOẠT ĐỘNG")
        log_title.setFont(QFont("Arial", 12, QFont.Bold))
        log_title.setAlignment(Qt.AlignCenter)
        layout.addWidget(log_title)
        
        # Text area cho log
        self.log_output = QTextEdit()
        self.log_output.setStyleSheet("""
            QTextEdit {
                background-color: #2b2b2b;
                color: #ffffff;
                border: 2px solid #444444;
                border-radius: 8px;
                font-family: 'Consolas', 'Monaco', monospace;
                font-size: 11px;
                padding: 10px;
            }
        """)
        layout.addWidget(self.log_output)
        
        # Buttons điều khiển log
        log_controls = QHBoxLayout()
        
        clear_log_btn = QPushButton("🗑️ XÓA LOG")
        clear_log_btn.setStyleSheet("""
            QPushButton {
                background-color: #ffc107;
                color: #212529;
                font-weight: bold;
                border-radius: 5px;
                padding: 8px 16px;
            }
            QPushButton:hover {
                background-color: #e0a800;
            }
        """)
        
        log_controls.addWidget(clear_log_btn)
        log_controls.addStretch()
        layout.addLayout(log_controls)
        
        # Kết nối signal
        clear_log_btn.clicked.connect(self.log_output.clear)
        
        return panel
    
    def setup_connections(self):
        """Thiết lập kết nối signals"""
        # SLAM controller signals
        self.slam_controller.output_received.connect(self.add_log)
        self.slam_controller.exploration_completed.connect(self.on_exploration_completed)
        self.slam_controller.slam_started.connect(self.on_slam_started)
        self.slam_controller.slam_stopped.connect(self.on_slam_stopped)
        
        # RViz controller signals
        self.rviz_controller.rviz_started.connect(self.on_rviz_started)
        self.rviz_controller.rviz_stopped.connect(self.on_rviz_stopped)
        self.rviz_controller.status_changed.connect(self.add_log)
        
        # Button signals
        self.start_slam_btn.clicked.connect(self.start_slam)
        self.stop_slam_btn.clicked.connect(self.stop_slam)
        self.start_rviz_btn.clicked.connect(self.start_rviz)
        self.stop_rviz_btn.clicked.connect(self.stop_rviz)
        self.save_map_btn.clicked.connect(self.save_map)
        self.load_map_btn.clicked.connect(self.load_map)
        self.exit_btn.clicked.connect(self.close_application)
    
    def apply_styles(self):
        """Áp dụng style cho ứng dụng"""
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f0f0f0;
            }
            QFrame {
                background-color: white;
                border: 1px solid #ddd;
                border-radius: 10px;
                margin: 5px;
                padding: 10px;
            }
            QLabel {
                color: #333;
                margin: 2px 0px;
            }
        """)
    
    def keyPressEvent(self, event):
        """Xử lý phím nhấn - vô hiệu hóa Ctrl hoàn toàn"""
        # Chặn tất cả tổ hợp phím với Ctrl
        if event.modifiers() & Qt.ControlModifier:
            event.ignore()
            return
        # Chặn phím Escape
        if event.key() == Qt.Key_Escape:
            event.ignore()
            return
        super().keyPressEvent(event)
    
    def start_slam(self):
        """Bắt đầu SLAM"""
        self.add_log("🚀 Đang khởi động SLAM...")
        if self.slam_controller.start_slam():
            self.start_slam_btn.setEnabled(False)
            self.stop_slam_btn.setEnabled(True)
            self.slam_progress.setVisible(True)
            self.slam_progress.setRange(0, 0)
        else:
            self.add_log("❌ Không thể khởi động SLAM!")
    
    def stop_slam(self):
        """Dừng SLAM"""
        self.add_log("⏹️ Đang dừng SLAM...")
        self.slam_controller.stop_slam()
    
    def start_rviz(self):
        """Khởi động RViz"""
        self.add_log("🚀 Đang khởi động RViz...")
        
        # Tạo config file nếu chưa có
        if not os.path.exists(self.config_path):
            self.create_rviz_config()
        
        if self.rviz_controller.start_rviz(self.config_path):
            self.start_rviz_btn.setEnabled(False)
            self.stop_rviz_btn.setEnabled(True)
        else:
            self.add_log("❌ Không thể khởi động RViz!")
    
    def stop_rviz(self):
        """Dừng RViz"""
        self.add_log("❌ Đang đóng RViz...")
        self.rviz_controller.stop_rviz()
    
    def save_map(self):
        """Lưu map hiện tại"""
        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            map_dir = os.path.join(script_dir, 'map_slam')
            os.makedirs(map_dir, exist_ok=True)
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            map_name = f"map_{timestamp}"
            map_path = os.path.join(map_dir, map_name)
            
            self.add_log(f"💾 Đang lưu map: {map_name}")
            
            result = subprocess.run([
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', map_path
            ], capture_output=True, text=True, timeout=30)
            
            if result.returncode == 0:
                self.add_log(f"✅ Map đã được lưu thành công: {map_name}")
            else:
                self.add_log(f"❌ Lỗi lưu map: {result.stderr}")
                
        except Exception as e:
            self.add_log(f"❌ Lỗi lưu map: {str(e)}")
    
    def load_map(self):
        """Tải map đã lưu"""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        map_dir = os.path.join(script_dir, 'map_slam')
        
        if not os.path.exists(map_dir):
            self.add_log("❌ Chưa có map nào được lưu")
            return
        
        map_files = [f for f in os.listdir(map_dir) if f.endswith('.yaml')]
        
        if not map_files:
            self.add_log("❌ Không tìm thấy map nào")
            return
        
        from PyQt5.QtWidgets import QInputDialog
        
        map_name, ok = QInputDialog.getItem(
            self, 'Chọn Map', 'Chọn map để tải:', 
            map_files, 0, False
        )
        
        if ok and map_name:
            self.add_log(f"📂 Đã chọn map: {map_name}")
    
    def create_rviz_config(self):
        """Tạo file config RViz"""
        os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
        
        config_content = '''Panels:
  - Class: rviz_common/Displays
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /LaserScan1
        - /Map1
      Splitter Ratio: 0.5
    Tree Height: 559

Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true

    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: 255; 255; 255
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 0
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: LaserScan
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.01
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /scan
      Use Fixed Frame: true
      Use rainbow: true
      Value: true

    - Alpha: 0.7
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: Map
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map
      Update Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map_updates
      Use Timestamp: false
      Value: true

  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: odom
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 10
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 1.57
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz_default_plugins)
      Yaw: 0

Window Geometry:
  Displays:
    collapsed: false
  Height: 846
  Hide Left Dock: false
  Hide Right Dock: false
  Width: 1200
  X: 100
  Y: 100'''
        
        with open(self.config_path, 'w') as f:
            f.write(config_content)
        
        self.add_log(f"📝 Đã tạo config RViz: {os.path.basename(self.config_path)}")
    
    def update_status(self):
        """Cập nhật trạng thái hệ thống"""
        # Cập nhật trạng thái ROS2
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                  capture_output=True, text=True, timeout=3)
            if result.returncode == 0:
                self.ros_status.setText("🟢 ROS2: Hoạt động")
            else:
                self.ros_status.setText("🔴 ROS2: Lỗi")
        except:
            self.ros_status.setText("🔴 ROS2: Không kết nối")
    
    def on_slam_started(self):
        """Xử lý khi SLAM bắt đầu"""
        self.slam_status.setText("🟢 SLAM: Đang chạy")
        self.add_log("✅ SLAM đã khởi động thành công!")
    
    def on_slam_stopped(self):
        """Xử lý khi SLAM dừng"""
        self.slam_status.setText("🔴 SLAM: Đã dừng")
        self.start_slam_btn.setEnabled(True)
        self.stop_slam_btn.setEnabled(False)
        self.slam_progress.setVisible(False)
        self.add_log("⏹️ SLAM đã dừng")
    
    def on_rviz_started(self):
        """Xử lý khi RViz bắt đầu"""
        self.rviz_status.setText("🟢 RViz: Đang chạy")
        self.add_log("✅ RViz đã khởi động thành công!")
    
    def on_rviz_stopped(self):
        """Xử lý khi RViz dừng"""
        self.rviz_status.setText("🔴 RViz: Đã đóng")
        self.start_rviz_btn.setEnabled(True)
        self.stop_rviz_btn.setEnabled(False)
        self.add_log("❌ RViz đã đóng")
    
    def on_exploration_completed(self):
        """Xử lý khi exploration hoàn thành"""
        self.add_log("🎉 KHÁM PHÁ ĐÃ HOÀN THÀNH!")
        
        reply = QMessageBox.question(
            self, 'Khám phá hoàn thành',
            'Quá trình khám phá đã hoàn thành!\nBạn có muốn lưu map không?',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.Yes
        )
        
        if reply == QMessageBox.Yes:
            self.save_map()
    
    def add_log(self, message):
        """Thêm message vào log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        formatted_message = f"[{timestamp}] {message}"
        self.log_output.append(formatted_message)
        
        # Tự động cuộn xuống cuối
        scrollbar = self.log_output.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    
    def close_application(self):
        """Đóng ứng dụng"""
        reply = QMessageBox.question(
            self, 'Xác nhận thoát',
            'Bạn có chắc muốn thoát?\nTất cả các tiến trình sẽ được dừng.',
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            self.close()
    
    def closeEvent(self, event):
        """Xử lý khi đóng cửa sổ"""
        self.add_log("🚪 Đang thoát ứng dụng...")
        
        # Dừng tất cả controllers
        self.slam_controller.stop_slam()
        self.rviz_controller.stop_rviz()
        
        # Đợi threads kết thúc
        if self.slam_controller.isRunning():
            self.slam_controller.wait(2000)
        if self.rviz_controller.isRunning():
            self.rviz_controller.wait(2000)
        
        event.accept()

def main():
    # Vô hiệu hóa signal handlers
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    
    app = QApplication(sys.argv)
    app.setAttribute(Qt.AA_DisableWindowContextHelpButton)
    
    # Xử lý Ctrl+C cho Qt
    def signal_handler(signum, frame):
        app.quit()
    
    signal.signal(signal.SIGINT, signal_handler)
    
    window = RVizGUI()
    window.show()
    
    window.add_log("🚀 Ứng dụng ROS2 SLAM Controller đã khởi động")
    window.add_log("💡 Tip: RViz có thể hoạt động ổn định khi không nhấn Ctrl")
    
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        window.close()

if __name__ == '__main__':
    main()