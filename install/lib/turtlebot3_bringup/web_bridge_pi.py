#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
import std_srvs.srv
import asyncio
import websockets
import websockets.exceptions
import json
import threading
import math
import time
import subprocess
import re
import os
import signal
from collections import deque

# GPIO control cho Pi Zero 2W - Define as module level
GPIO_AVAILABLE = False
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO = None
    print("⚠️  RPi.GPIO not available - GPIO control disabled")

def euler_from_quaternion(x, y, z, w):
    """
    Chuyển đổi Quaternion (dữ liệu định hướng của ROS) sang góc Euler (yaw).
    """
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

class WebBridgePi:
    """
    Phiên bản tối ưu cho Raspberry Pi Zero 2W với throttling và error handling.
    
    Tối ưu hóa cho Pi Zero 2W (512MB RAM + 2GB Swap, 1GHz CPU):
    - Map throttle: 2s (cân bằng performance/resource)  
    - Data limit: 30k cells (tận dụng swap 2GB)
    - Odom throttle: 0.3s (nhanh hơn với swap support)
    - Log streaming: Filtered + throttled (50 logs buffer)
    - Reduced logging (tiết kiệm I/O)
    """
    def __init__(self, node: Node):
        # --- FIX: Khai báo sử dụng biến global ---
        global GPIO_AVAILABLE

        self._node = node
        self._logger = node.get_logger()
        
        # --- Cấu hình tối ưu cho Pi ---
        self.port = 8765
        self.host = "0.0.0.0"  # Cho phép kết nối từ mạng
        self.clients = set()
        self.server_loop = None
        
        # --- Throttling tối ưu cho Pi Zero 2W + Swap 2GB ---
        self.last_map_time = 0
        self.last_odom_time = 0
        self.map_throttle = 2.0  # Giây - Vừa phải với swap 2GB
        self.odom_throttle = 0.3  # Giây - Nhanh hơn với swap support
        
        # --- Lưu trữ dữ liệu ---
        self.last_map_data = None
        self.last_odom_data = None
        
        # --- Log monitoring cho Pi Zero 2W ---
        self.enable_log_streaming = True  # Set False để tắt log streaming
        self.log_buffer = deque(maxlen=50)  # Giới hạn 50 logs để tiết kiệm RAM
        self.last_log_time = 0
        self.log_throttle = 1.0  # 1 giây throttle cho logs
        self.important_keywords = [
            'ERROR', 'WARN', 'failed', 'disconnect', 'connect', 
            'IMU', 'Motor', 'SLAM', 'Map', 'Battery', 'Init', 'Run'
        ]
        
        # --- GPIO Control cho Pi Zero 2W ---
        self.motor_pin = 27  # GPIO27 (Pin 13 physical/BOARD mode) = GPIO27 BCM mode
        self.motor_enabled = False
        self.motor_control_pin = 25  # GPIO25 (Pin 22 physical/BOARD mode) = GPIO25 BCM mode
        self.motor_control_enabled = False
        
        # --- LiDAR PWM Control ---
        self.lidar_pwm_pin = 18  # GPIO18 (Pin 12 BOARD mode in pwm.py) = GPIO18 BCM mode
        self.lidar_pwm = None
        self.lidar_enabled = False  # Start with LiDAR DISABLED (no rotation)
        self.lidar_duty_cycle = 7  # Working duty cycle for LiDAR when enabled
        
        # --- Frontier Exploration Control ---
        self.exploration_enabled = False  # Start with exploration DISABLED
        self.exploration_process = None
        
        # --- ROS Service Clients ---
        self.start_exploration_client = None
        self.stop_exploration_client = None
        
        # --- Process Control ---
        self.lidar_process = None
        self.slam_process = None
        
        # --- Setup GPIO ---
        if GPIO_AVAILABLE:
            try:
                # Stop any existing pwm.py process that might conflict
                try:
                    subprocess.run(["pkill", "-f", "pwm.py"], capture_output=True, timeout=2)
                    self._logger.info("🔄 Stopped existing pwm.py process to avoid GPIO conflict")
                except:
                    pass
                
                GPIO.setmode(GPIO.BCM)
                
                # Setup GPIO27 for motor power control (Pin 13 physical/BOARD mode)
                GPIO.setup(self.motor_pin, GPIO.OUT)
                GPIO.output(self.motor_pin, GPIO.LOW)  # Start with motor OFF
                self._logger.info(f"✅ Motor Power GPIO{self.motor_pin} (Pin 13 physical) initialized (LOW)")
                
                # Setup GPIO25 for motor control (Pin 22 physical/BOARD mode)
                GPIO.setup(self.motor_control_pin, GPIO.OUT)
                GPIO.output(self.motor_control_pin, GPIO.LOW)  # Start with motor control OFF
                self._logger.info(f"✅ Motor Control GPIO{self.motor_control_pin} (Pin 22 physical) initialized (LOW)")
                
                # Setup GPIO18 for LiDAR PWM control (same as Pin 12 BOARD in pwm.py)
                GPIO.setup(self.lidar_pwm_pin, GPIO.OUT)
                self.lidar_pwm = GPIO.PWM(self.lidar_pwm_pin, 100)  # 100Hz frequency
                
                # Start with LiDAR OFF (0% duty cycle)
                initial_duty = 0 if not self.lidar_enabled else self.lidar_duty_cycle
                self.lidar_pwm.start(initial_duty)
                status = "OFF (0%)" if not self.lidar_enabled else f"ON ({self.lidar_duty_cycle}%)"
                self._logger.info(f"✅ LiDAR PWM GPIO{self.lidar_pwm_pin} initialized - {status}")
                
            except Exception as e:
                self._logger.error(f"❌ GPIO setup failed: {e}")
                GPIO_AVAILABLE = False
        else:
            self._logger.warning("⚠️  GPIO not available - motor and LiDAR control disabled")
        
        self._logger.info("🚀 Khởi tạo Web Bridge cho Pi Zero 2W + Swap 2GB...")

    def start(self):
        """Khởi động các thành phần của bridge."""
        try:
            # --- Tạo Subscribers ---
            self._node.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
            self._node.create_subscription(Odometry, '/odom', self.odom_callback, 10)
            
            # --- Setup cmd_vel publisher for manual control ---
            self.cmd_vel_publisher = self._node.create_publisher(Twist, '/cmd_vel', 10)
            self._logger.info("✅ cmd_vel publisher initialized for manual control")
            
            # --- Setup service clients for exploration control ---
            import std_srvs.srv
            self.start_exploration_client = self._node.create_client(std_srvs.srv.Trigger, '/start_exploration')
            self.stop_exploration_client = self._node.create_client(std_srvs.srv.Trigger, '/stop_exploration')
            self._logger.info("✅ Exploration service clients initialized")
            
            # --- Khởi động WebSocket Server trong một thread riêng ---
            server_thread = threading.Thread(target=self.start_server_thread, daemon=True)
            server_thread.start()
            
            # --- Khởi động Log Monitoring nếu được bật ---
            if self.enable_log_streaming:
                self.start_log_monitoring()
            
            self._logger.info(f"✅ Web Bridge đã khởi động thành công")
        except Exception as e:
            self._logger.error(f"❌ Lỗi khởi động Web Bridge: {e}")
    
    def start_log_monitoring(self):
        """Bắt đầu monitor ROS2 logs nếu được bật."""
        if not self.enable_log_streaming:
            return
            
        log_thread = threading.Thread(target=self.monitor_node_logs, daemon=True)
        log_thread.start()
        self._logger.info("📋 Đã khởi động log monitoring thread")
    
    def filter_important_log(self, log_line):
        """Filter logs quan trọng để giảm tải Pi Zero 2W."""
        if not log_line.strip():
            return False
            
        # Chỉ gửi logs có keywords quan trọng
        for keyword in self.important_keywords:
            if keyword.lower() in log_line.lower():
                return True
        return False
    
    def add_log_to_buffer(self, log_line):
        """Thêm log vào buffer với throttling."""
        current_time = time.time()
        if current_time - self.last_log_time < self.log_throttle:
            return
            
        if self.filter_important_log(log_line):
            self.last_log_time = current_time
            
            # Clean và format log
            clean_log = re.sub(r'\[.*?\]', '', log_line).strip()
            timestamp = time.strftime('%H:%M:%S')
            
            log_entry = {
                'type': 'log',
                'timestamp': current_time,
                'time_str': timestamp,
                'message': clean_log[:200]  # Limit message length
            }
            
            self.log_buffer.append(log_entry)
            self.broadcast(log_entry)
    
    def monitor_node_logs(self):
        """Monitor logs từ ROS nodes - simplified version cho Pi Zero 2W."""
        try:
            # Đơn giản hóa: chỉ monitor current node logs
            self._logger.info("📊 Log monitoring started (simplified for Pi Zero 2W)")
            
            # Thêm một số system logs quan trọng
            import os
            import glob
            
            while True:
                try:
                    # Check ROS log directory
                    ros_log_dir = os.path.expanduser("~/.ros/log")
                    if os.path.exists(ros_log_dir):
                        # Tìm log files mới nhất
                        log_dirs = glob.glob(f"{ros_log_dir}/*/")
                        if log_dirs:
                            latest_dir = max(log_dirs, key=os.path.getctime)
                            
                            # Monitor một vài file log quan trọng
                            important_logs = [
                                'turtlebot3_ros-*/stdout.log',
                                'lds02rr_node-*/stdout.log'
                            ]
                            
                            for pattern in important_logs:
                                log_files = glob.glob(f"{latest_dir}{pattern}")
                                for log_file in log_files:
                                    try:
                                        # Đọc last few lines
                                        with open(log_file, 'r') as f:
                                            lines = f.readlines()
                                            if lines:
                                                last_line = lines[-1].strip()
                                                if last_line and len(last_line) > 10:
                                                    self.add_log_to_buffer(last_line)
                                    except Exception:
                                        pass  # Ignore file read errors
                    
                    time.sleep(2)  # Check mỗi 2 giây để tiết kiệm CPU
                    
                except Exception:
                    time.sleep(5)  # Longer sleep on error
                    
        except Exception as e:
            self._logger.warning(f"⚠️  Log monitoring error: {e}")

    def start_server_thread(self):
        """Hàm target cho thread của server, thiết lập và chạy event loop."""
        try:
            self.server_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.server_loop)
            
            self.server_loop.run_until_complete(self.server_main())
        except Exception as e:
            self._logger.error(f"❌ Lỗi server thread: {e}")
        finally:
            if self.server_loop:
                self.server_loop.close()

    async def server_main(self):
        """Logic chính của server, chạy vô tận."""
        try:
            self._logger.info(f"🛰️  WebSocket Server đang lắng nghe trên {self.host}:{self.port}")
            async with websockets.serve(self.handler, self.host, self.port):
                await asyncio.Future()
        except Exception as e:
            self._logger.error(f"❌ Lỗi WebSocket server: {e}")

    async def handler(self, websocket):
        """Xử lý các kết nối mới từ client."""
        client_ip = websocket.remote_address[0] if websocket.remote_address else "unknown"
        self._logger.info(f"✅ Client mới kết nối từ {client_ip}")
        
        self.clients.add(websocket)
        try:
            # Gửi dữ liệu hiện tại ngay khi client kết nối
            if self.last_map_data or self.last_odom_data or self.log_buffer:
                initial_data = {
                    'map': self.last_map_data,
                    'odom': self.last_odom_data,
                    'logs': list(self.log_buffer) if self.enable_log_streaming else [],
                    'timestamp': time.time()
                }
                await websocket.send(json.dumps(initial_data))
                self._logger.info(f"📤 Đã gửi dữ liệu ban đầu cho {client_ip} (bao gồm {len(self.log_buffer)} logs)")

            # Giữ kết nối mở và lắng nghe messages tùy chọn
            try:
                async for message in websocket:
                    try:
                        data = json.loads(message)
                        if data.get('action') == 'toggle_logs':
                            self.toggle_log_streaming()
                        elif data.get('action') == 'ping':
                            # Respond to ping để maintain connection
                            await websocket.send(json.dumps({'type': 'pong', 'timestamp': time.time()}))
                        elif data.get('action') == 'control_lidar':
                            # Handle LiDAR control
                            enabled = data.get('enabled', False)
                            success, message = self.control_lidar(enabled)
                            response = {
                                'type': 'control_response',
                                'action': 'control_lidar',
                                'success': success,
                                'enabled': enabled if success else self.lidar_enabled,
                                'message': message,
                                'timestamp': time.time()
                            }
                            await websocket.send(json.dumps(response))
                        elif data.get('action') == 'control_motor':
                            # Handle Motor control
                            enabled = data.get('enabled', False)
                            pin = data.get('pin', 27)  # Default to GPIO27 for backward compatibility
                            success, message = self.control_motor(enabled, pin)
                            
                            # Determine which state to return based on pin
                            if pin == 27:
                                current_state = self.motor_enabled
                            elif pin == 25:
                                current_state = self.motor_control_enabled
                            else:
                                current_state = False
                                
                            response = {
                                'type': 'control_response',
                                'action': 'control_motor',
                                'success': success,
                                'enabled': enabled if success else current_state,
                                'pin': pin,
                                'message': message,
                                'timestamp': time.time()
                            }
                            await websocket.send(json.dumps(response))
                        elif data.get('action') == 'control_exploration':
                            # Handle Frontier Exploration control
                            enabled = data.get('enabled', False)
                            success, message = self.control_exploration(enabled)
                            response = {
                                'type': 'control_response',
                                'action': 'control_exploration',
                                'success': success,
                                'enabled': enabled if success else self.exploration_enabled,
                                'message': message,
                                'timestamp': time.time()
                            }
                            await websocket.send(json.dumps(response))
                        elif data.get('action') == 'send_cmd_vel':
                            # Handle manual velocity control
                            linear_x = data.get('linear_x', 0.0)
                            angular_z = data.get('angular_z', 0.0)
                            success, message = self.publish_cmd_vel(linear_x, angular_z)
                            
                            response = {
                                'type': 'control_response',
                                'action': 'send_cmd_vel',
                                'success': success,
                                'message': message,
                                'linear_x': linear_x,
                                'angular_z': angular_z,
                                'timestamp': time.time()
                            }
                            await websocket.send(json.dumps(response))
                    except json.JSONDecodeError:
                        self._logger.warning(f"⚠️  Invalid JSON from client {client_ip}")
                    except Exception as e:
                        self._logger.warning(f"⚠️  Error processing message from {client_ip}: {e}")
            except websockets.exceptions.ConnectionClosed:
                # Normal disconnection
                pass
        except websockets.exceptions.ConnectionClosed:
            self._logger.info(f"ℹ️  Client {client_ip} đã ngắt kết nối bình thường")
        except websockets.exceptions.ConnectionClosedError as e:
            self._logger.info(f"ℹ️  Client {client_ip} kết nối bị đóng: {e.code}")
        except Exception as e:
            self._logger.warning(f"⚠️  Lỗi kết nối với client {client_ip}: {e}")
        finally:
            self.clients.discard(websocket)
            self._logger.info(f"🔌 Client {client_ip} đã được xóa khỏi danh sách")

    def map_callback(self, msg):
        """Xử lý dữ liệu bản đồ với throttling."""
        current_time = time.time()
        if current_time - self.last_map_time < self.map_throttle:
            return
        
        self.last_map_time = current_time
        # Giảm logging để tiết kiệm CPU Pi Zero 2W
        
        try:
            # Chỉ lưu thông tin cơ bản để tiết kiệm memory
            map_info = {
                'type': 'map',
                'timestamp': current_time,
                'info': {
                    'resolution': float(msg.info.resolution),
                    'width': int(msg.info.width),
                    'height': int(msg.info.height),
                    'origin': {
                        'x': float(msg.info.origin.position.x), 
                        'y': float(msg.info.origin.position.y)
                    },
                },
                'data_size': len(msg.data),
                'data': list(msg.data) if len(msg.data) < 30000 else None  # Giới hạn 30k với Swap 2GB
            }
            
            if map_info['data'] is None:
                map_info['message'] = f"Map quá lớn ({len(msg.data)} cells), chỉ gửi metadata"
                # Chỉ log khi cần thiết để tiết kiệm CPU
                if len(msg.data) % 5000 == 0:  # Log mỗi 5k cells
                    self._logger.info(f"📊 Map size: {len(msg.data)} cells")
            
            self.last_map_data = map_info
            self.broadcast(map_info)
        except Exception as e:
            self._logger.error(f"❌ Lỗi xử lý map callback: {e}")
        
    def odom_callback(self, msg):
        """Xử lý dữ liệu odometry với throttling."""
        current_time = time.time()
        if current_time - self.last_odom_time < self.odom_throttle:
            return
            
        self.last_odom_time = current_time
        
        try:
            q = msg.pose.pose.orientation
            yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

            odom_info = {
                'type': 'odom',
                'timestamp': current_time,
                'pose': {
                    'position': {
                        'x': float(msg.pose.pose.position.x), 
                        'y': float(msg.pose.pose.position.y),
                        'z': float(msg.pose.pose.position.z)
                    },
                    'orientation_yaw': float(yaw)
                },
                'twist': {
                    'linear': {
                        'x': float(msg.twist.twist.linear.x),
                        'y': float(msg.twist.twist.linear.y),
                        'z': float(msg.twist.twist.linear.z)
                    },
                    'angular': {
                        'x': float(msg.twist.twist.angular.x),
                        'y': float(msg.twist.twist.angular.y), 
                        'z': float(msg.twist.twist.angular.z)
                    }
                }
            }
            self.last_odom_data = odom_info
            self.broadcast(odom_info)
        except Exception as e:
            self._logger.error(f"❌ Lỗi xử lý odom callback: {e}")
        
    def broadcast(self, message_data):
        """Gửi message tới tất cả các client."""
        if not self.clients:
            return
        
        try:
            message_json = json.dumps(message_data)
            
            # Đảm bảo rằng coroutine được chạy trên đúng event loop của server
            if self.server_loop and self.server_loop.is_running():
                asyncio.run_coroutine_threadsafe(self.send_to_all(message_json), self.server_loop)
        except Exception as e:
            self._logger.error(f"❌ Lỗi broadcast: {e}")

    async def send_to_all(self, message_json):
        """Hàm async để gửi dữ liệu."""
        if not self.clients:
            return
            
        disconnected_clients = set()
        tasks = []
        
        for client in self.clients.copy():  # Copy để tránh modification during iteration
            try:
                # Đơn giản hóa: chỉ dùng try/except thay vì check attributes
                tasks.append(client.send(message_json))
            except (websockets.exceptions.ConnectionClosed, 
                    websockets.exceptions.ConnectionClosedError,
                    websockets.exceptions.ConnectionClosedOK,
                    AttributeError) as e:
                # Client đã disconnect hoặc có lỗi attribute
                disconnected_clients.add(client)
            except Exception as e:
                # Only log non-timeout errors to reduce spam
                if "timeout" not in str(e).lower():
                    self._logger.warning(f"⚠️  Client error: {e}")
                disconnected_clients.add(client)
        
        # Remove disconnected clients
        for client in disconnected_clients:
            self.clients.discard(client)
            
        if tasks:
            results = await asyncio.gather(*tasks, return_exceptions=True)
            # Log errors but filter out common timeout issues
            error_count = 0
            for i, result in enumerate(results):
                if isinstance(result, Exception):
                    error_msg = str(result)
                    # Only log non-timeout errors or first few timeout errors
                    if "timeout" not in error_msg.lower() or error_count < 2:
                        self._logger.warning(f"⚠️  Error sending to client: {result}")
                        if "timeout" in error_msg.lower():
                            error_count += 1
                    
        # Only log when there are significant changes
        if disconnected_clients and len(disconnected_clients) > 0:
            self._logger.info(f"📤 Sent to {len(tasks)} clients, {len(disconnected_clients)} disconnected")
        # Không log success để tiết kiệm CPU Pi Zero 2W trừ khi debug
    
    def toggle_log_streaming(self, enable=None):
        """Toggle log streaming on/off để tiết kiệm tài nguyên."""
        if enable is not None:
            self.enable_log_streaming = enable
        else:
            self.enable_log_streaming = not self.enable_log_streaming
            
        status = "enabled" if self.enable_log_streaming else "disabled"
        self._logger.info(f"📋 Log streaming {status}")
        
        # Broadcast status change
        status_msg = {
            'type': 'log_status',
            'enabled': self.enable_log_streaming,
            'timestamp': time.time()
        }
        self.broadcast(status_msg)
        
        return self.enable_log_streaming

    # --- Control Methods ---
    def control_motor(self, enabled: bool, pin: int = 27):
        """Điều khiển motor qua GPIO pin (27 hoặc 25)."""
        if not GPIO_AVAILABLE:
            return False, "GPIO not available"
        
        try:
            if pin == 27:
                # Motor power control (GPIO27)
                GPIO.output(self.motor_pin, GPIO.HIGH if enabled else GPIO.LOW)
                self.motor_enabled = enabled
                status = "HIGH" if enabled else "LOW"
                self._logger.info(f"⚡ Motor Power: GPIO{self.motor_pin} (Pin 13 physical) set to {status}")
                return True, f"GPIO{self.motor_pin} (Pin 13 physical) set to {status}"
            elif pin == 25:
                # Motor control (GPIO25)
                GPIO.output(self.motor_control_pin, GPIO.HIGH if enabled else GPIO.LOW)
                self.motor_control_enabled = enabled
                status = "HIGH" if enabled else "LOW"
                self._logger.info(f"🔧 Motor Control: GPIO{self.motor_control_pin} (Pin 22 physical) set to {status}")
                return True, f"GPIO{self.motor_control_pin} (Pin 22 physical) set to {status}"
            else:
                return False, f"Invalid pin {pin} - only 27 and 25 supported"
        except Exception as e:
            self._logger.error(f"❌ Motor control failed: {e}")
            return False, str(e)
    
    def control_lidar(self, enabled: bool):
        """Điều khiển LiDAR rotation only (keep SLAM running)."""
        if enabled == self.lidar_enabled:
            return True, f"LiDAR already {'enabled' if enabled else 'disabled'}"
        
        try:
            if enabled:
                # Start LiDAR only (SLAM keeps running if already started)
                self._logger.info("🔴 Starting LiDAR...")
                success = self._start_lidar_only()
                if success:
                    self.lidar_enabled = True
                    return True, "LiDAR started successfully"
                else:
                    return False, "Failed to start LiDAR"
            else:
                # Stop LiDAR only (keep SLAM running)
                self._logger.info("🔴 Stopping LiDAR only...")
                self._stop_lidar_only()
                self.lidar_enabled = False
                
                # Send stop command to robot for safety
                self._send_stop_command()
                
                return True, "LiDAR stopped (SLAM continues running)"
                
        except Exception as e:
            self._logger.error(f"❌ LiDAR control failed: {e}")
            return False, str(e)

    def control_exploration(self, enabled: bool):
        """Điều khiển Frontier Exploration qua ROS service."""
        if enabled == self.exploration_enabled:
            return True, f"Exploration already {'enabled' if enabled else 'disabled'}"
        
        try:
            if enabled:
                # Start Frontier Exploration via service
                self._logger.info("🗺️ Starting Frontier Exploration via service...")
                success = self._start_exploration_service()
                if success:
                    self.exploration_enabled = True
                    return True, "Frontier Exploration started successfully"
                else:
                    return False, "Failed to start Frontier Exploration"
            else:
                # Stop Frontier Exploration via service
                self._logger.info("🗺️ Stopping Frontier Exploration via service...")
                success = self._stop_exploration_service()
                if success:
                    self.exploration_enabled = False
                    return True, "Frontier Exploration stopped successfully"
                else:
                    return False, "Failed to stop Frontier Exploration"
                
        except Exception as e:
            self._logger.error(f"❌ Exploration control failed: {e}")
            return False, str(e)
    
    def _start_lidar_slam(self):
        """Start LiDAR và SLAM processes - Optimized for Cartographer."""
        try:
            self._logger.info("🔄 Starting LiDAR and Cartographer SLAM...")
            
            # Method 1: Start LiDAR node specifically
            lidar_start_cmd = ["ros2", "launch", "lds02rr_lidar", "lds02rr.launch.py", "port:=/dev/ttyAMA0"]
            
            # Run in background
            lidar_process = subprocess.Popen(
                lidar_start_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Create process group for cleanup
            )
            
            # Wait a bit for LiDAR to start
            time.sleep(2)
            
            # Check if LiDAR process is still running
            if lidar_process.poll() is None:
                self.lidar_process = lidar_process
                self._logger.info("✅ LiDAR started successfully")
                
                # Start Cartographer SLAM
                return self._start_cartographer()
            else:
                stderr_output = lidar_process.stderr.read().decode()
                self._logger.error(f"❌ LiDAR failed to start: {stderr_output}")
                return False
                
        except Exception as e:
            self._logger.error(f"❌ LiDAR start error: {e}")
            return False
    
    def _start_cartographer(self):
        """Start Cartographer SLAM separately."""
        try:
            self._logger.info("🗺️  Starting Cartographer SLAM...")
            
            # Start Cartographer without RViz
            cartographer_cmd = [
                "ros2", "launch", "turtlebot3_cartographer", "cartographer.launch.py",
                "use_sim_time:=false",
                "use_rviz:=false"
            ]
            
            # Run in background
            slam_process = subprocess.Popen(
                cartographer_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            
            # Wait for SLAM to initialize
            time.sleep(3)
            
            if slam_process.poll() is None:
                self.slam_process = slam_process
                self._logger.info("✅ Cartographer SLAM started successfully")
                return True
            else:
                stderr_output = slam_process.stderr.read().decode()
                self._logger.error(f"❌ Cartographer failed to start: {stderr_output}")
                return False
                
        except Exception as e:
            self._logger.error(f"❌ Cartographer start error: {e}")
            return False
    
    def _start_lidar_only(self):
        """Start LiDAR PWM rotation (keep ROS node running)."""
        try:
            self._logger.info("🔄 Starting LiDAR PWM rotation...")
            
            if not GPIO_AVAILABLE or not self.lidar_pwm:
                return False, "GPIO PWM not available"
            
            # Start PWM with default duty cycle to spin LiDAR
            self.lidar_pwm.ChangeDutyCycle(self.lidar_duty_cycle)
            self._logger.info(f"✅ LiDAR PWM started ({self.lidar_duty_cycle}% duty cycle)")
            
            # Small delay to ensure PWM is active
            time.sleep(0.5)
            
            return True
                
        except Exception as e:
            self._logger.error(f"❌ LiDAR PWM start error: {e}")
            return False
    
    def _stop_lidar_only(self):
        """Stop LiDAR PWM rotation (keep ROS node running)."""
        try:
            self._logger.info("🛑 Stopping LiDAR PWM rotation (keeping ROS node & SLAM running)...")
            
            if not GPIO_AVAILABLE or not self.lidar_pwm:
                self._logger.warning("⚠️  GPIO PWM not available")
                return
            
            # Stop PWM (0% duty cycle = no rotation)
            self.lidar_pwm.ChangeDutyCycle(0)
            self._logger.info("✅ LiDAR PWM stopped (0% duty cycle)")
            
            # Small delay to ensure PWM is stopped
            time.sleep(0.5)
            
            self._logger.info("✅ LiDAR rotation stopped (ROS node & SLAM continue running)")
            
        except Exception as e:
            self._logger.error(f"❌ Error stopping LiDAR PWM: {e}")
    
    def _stop_lidar_slam(self):
        """Stop LiDAR và SLAM processes - Clean shutdown."""
        try:
            self._logger.info("🛑 Stopping LiDAR and SLAM...")
            
            # Stop Cartographer SLAM first
            if self.slam_process and self.slam_process.poll() is None:
                try:
                    self._logger.info("🗺️  Stopping Cartographer SLAM...")
                    os.killpg(os.getpgid(self.slam_process.pid), signal.SIGTERM)
                    
                    # Wait for graceful shutdown
                    try:
                        self.slam_process.wait(timeout=5)
                        self._logger.info("✅ Cartographer stopped gracefully")
                    except subprocess.TimeoutExpired:
                        self._logger.warning("⚠️  Force killing Cartographer...")
                        os.killpg(os.getpgid(self.slam_process.pid), signal.SIGKILL)
                        
                except Exception as e:
                    self._logger.error(f"❌ Error stopping Cartographer: {e}")
                finally:
                    self.slam_process = None
            
            # Stop LiDAR process
            if self.lidar_process and self.lidar_process.poll() is None:
                try:
                    self._logger.info("🔴 Stopping LiDAR...")
                    os.killpg(os.getpgid(self.lidar_process.pid), signal.SIGTERM)
                    
                    # Wait for graceful shutdown
                    try:
                        self.lidar_process.wait(timeout=5)
                        self._logger.info("✅ LiDAR stopped gracefully")
                    except subprocess.TimeoutExpired:
                        self._logger.warning("⚠️  Force killing LiDAR...")
                        os.killpg(os.getpgid(self.lidar_process.pid), signal.SIGKILL)
                        
                except Exception as e:
                    self._logger.error(f"❌ Error stopping LiDAR: {e}")
                finally:
                    self.lidar_process = None
            
            # Fallback: Kill any remaining processes
            try:
                subprocess.run(["pkill", "-f", "lds02rr"], capture_output=True, timeout=2)
                subprocess.run(["pkill", "-f", "cartographer"], capture_output=True, timeout=2)
            except:
                pass
            
            self._logger.info("✅ LiDAR and SLAM stopped")
        except Exception as e:
            self._logger.error(f"❌ Error stopping LiDAR/SLAM: {e}")

    def _start_exploration_service(self):
        """Start Frontier Exploration via ROS service."""
        try:
            self._logger.info("🗺️ Calling start_exploration service...")
            
            if not self.start_exploration_client:
                self._logger.error("❌ Start exploration service client not initialized")
                return False
            
            # Check if service exists
            self._logger.info("🔍 Checking if start_exploration service exists...")
            
            # Wait for service to be available
            if not self.start_exploration_client.wait_for_service(timeout_sec=10.0):
                self._logger.error("❌ Start exploration service not available after 10 seconds")
                self._logger.error("🔍 Available services:")
                try:
                    # List available services for debugging
                    import subprocess
                    result = subprocess.run(['ros2', 'service', 'list'], capture_output=True, text=True, timeout=5)
                    if result.stdout:
                        services = result.stdout.strip().split('\n')
                        exploration_services = [s for s in services if 'exploration' in s.lower()]
                        self._logger.info(f"🔍 Found exploration services: {exploration_services}")
                    else:
                        self._logger.error("❌ No services found or ros2 command failed")
                except Exception as e:
                    self._logger.error(f"❌ Error listing services: {e}")
                return False
            
            self._logger.info("✅ Start exploration service is available")
            
            # Call service
            request = std_srvs.srv.Trigger.Request()
            self._logger.info("📤 Sending start_exploration service request...")
            future = self.start_exploration_client.call_async(request)
            
            # Wait for response
            self._logger.info("⏳ Waiting for service response...")
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)
            
            if future.done():
                response = future.result()
                self._logger.info(f"📥 Service response received: success={response.success}, message='{response.message}'")
                if response.success:
                    self._logger.info(f"✅ Frontier Exploration started: {response.message}")
                    return True
                else:
                    self._logger.warning(f"⚠️ Frontier Exploration start failed: {response.message}")
                    return False
            else:
                self._logger.error("❌ Start exploration service call timeout")
                return False
                
        except Exception as e:
            self._logger.error(f"❌ Start exploration service error: {e}")
            import traceback
            self._logger.error(f"🔍 Full traceback: {traceback.format_exc()}")
            return False

    def _stop_exploration_service(self):
        """Stop Frontier Exploration via ROS service."""
        try:
            self._logger.info("🗺️ Calling stop_exploration service...")
            
            if not self.stop_exploration_client:
                self._logger.error("❌ Stop exploration service client not initialized")
                return False
            
            # Wait for service to be available
            if not self.stop_exploration_client.wait_for_service(timeout_sec=5.0):
                self._logger.error("❌ Stop exploration service not available")
                return False
            
            # Call service
            request = std_srvs.srv.Trigger.Request()
            future = self.stop_exploration_client.call_async(request)
            
            # Wait for response
            rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
            
            if future.done():
                response = future.result()
                if response.success:
                    self._logger.info(f"✅ Frontier Exploration stopped: {response.message}")
                    return True
                else:
                    self._logger.warning(f"⚠️ Frontier Exploration stop failed: {response.message}")
                    return False
            else:
                self._logger.error("❌ Stop exploration service call timeout")
                return False
                
        except Exception as e:
            self._logger.error(f"❌ Stop exploration service error: {e}")
            return False
    
    def _send_stop_command(self):
        """Gửi lệnh dừng robot để safety khi LiDAR tắt."""
        try:
            # Create a simple publisher để send stop command
            stop_pub = self._node.create_publisher(Twist, '/cmd_vel', 10)
            stop_msg = Twist()  # All velocities = 0
            
            # Send stop command multiple times để ensure safety
            for i in range(10):  # Increased for better safety
                stop_pub.publish(stop_msg)
                time.sleep(0.05)
                
            self._logger.info("🛑 Emergency stop command sent to robot")
            
            # Broadcast safety alert to web clients
            safety_alert = {
                'type': 'safety_alert',
                'message': 'Robot stopped - LiDAR rotation disabled (SLAM continues)',
                'level': 'warning',
                'timestamp': time.time()
            }
            self.broadcast(safety_alert)
            
        except Exception as e:
            self._logger.error(f"❌ Failed to send stop command: {e}")
            
            # Emergency fallback: disable motor power and control if possible
            if GPIO_AVAILABLE:
                try:
                    if self.motor_enabled:
                        GPIO.output(self.motor_pin, GPIO.LOW)
                        self.motor_enabled = False
                        self._logger.info(f"⚡ Emergency: Motor power disabled via GPIO{self.motor_pin} (Pin 13 physical)")
                    
                    if self.motor_control_enabled:
                        GPIO.output(self.motor_control_pin, GPIO.LOW)
                        self.motor_control_enabled = False
                        self._logger.info(f"🔧 Emergency: Motor control disabled via GPIO{self.motor_control_pin} (Pin 22 physical)")
                except:
                    pass

    def publish_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        """
        Publish velocity command for manual robot control.
        
        Args:
            linear_x (float): Linear velocity in m/s (max 0.1 m/s)
            angular_z (float): Angular velocity in rad/s
        """
        try:
            # Safety limits - Max 0.1 m/s linear, reasonable angular
            linear_x = max(-0.1, min(0.1, linear_x))
            angular_z = max(-2.0, min(2.0, angular_z))
            
            cmd_msg = Twist()
            cmd_msg.linear.x = float(linear_x)
            cmd_msg.linear.y = 0.0
            cmd_msg.linear.z = 0.0
            cmd_msg.angular.x = 0.0
            cmd_msg.angular.y = 0.0
            cmd_msg.angular.z = float(angular_z)
            
            self.cmd_vel_publisher.publish(cmd_msg)
            
            # Reduce logging frequency to avoid spam - only log when velocity changes significantly
            if not hasattr(self, '_last_linear') or not hasattr(self, '_last_angular'):
                self._last_linear = 0.0
                self._last_angular = 0.0
            
            if abs(linear_x - self._last_linear) > 0.01 or abs(angular_z - self._last_angular) > 0.1:
                self._logger.info(f"🎮 Manual control: linear={linear_x:.3f} m/s, angular={angular_z:.3f} rad/s")
                self._last_linear = linear_x
                self._last_angular = angular_z
            
            return True, "Velocity command sent successfully"
            
        except Exception as e:
            self._logger.error(f"❌ Failed to publish cmd_vel: {e}")
            return False, f"Failed to send velocity command: {e}"

    def cleanup(self):
        """Cleanup GPIO and processes khi shutdown."""
        try:
            if GPIO_AVAILABLE:
                self._logger.info("🧹 Cleaning up GPIO...")
                
                # Turn off motor power
                GPIO.output(self.motor_pin, GPIO.LOW)
                
                # Turn off motor control
                GPIO.output(self.motor_control_pin, GPIO.LOW)
                
                # Stop LiDAR PWM
                if self.lidar_pwm:
                    self.lidar_pwm.ChangeDutyCycle(0)  # Stop rotation
                    self.lidar_pwm.stop()              # Stop PWM
                    self._logger.info(f"🔴 LiDAR PWM GPIO{self.lidar_pwm_pin} stopped and cleaned up")
                
                GPIO.cleanup()
                
            # Stop LiDAR/SLAM processes if running
            if self.lidar_process or self.slam_process:
                self._stop_lidar_slam()
            
            # Stop Frontier Exploration if running
            if self.exploration_enabled:
                self._stop_exploration_service()
                
            self._logger.info("✅ Cleanup completed")
            
        except Exception as e:
            self._logger.error(f"❌ Cleanup error: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    # Tạo node ROS2
    node = rclpy.create_node('web_bridge_pi_node')
    
    # Tạo và khởi động bridge
    bridge = WebBridgePi(node)
    bridge.start()
    
    try:
        # Giữ cho node ROS hoạt động
        node.get_logger().info("🔄 Web Bridge Pi đang chạy...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] Tắt cầu nối Web...")
    finally:
        bridge.cleanup()  # Cleanup GPIO and processes
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
