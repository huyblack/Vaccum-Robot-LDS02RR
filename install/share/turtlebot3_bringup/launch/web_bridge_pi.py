#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import asyncio
import websockets
import websockets.exceptions
import json
import threading
import math
import time
import subprocess
import os
import signal
from collections import deque

# GPIO control cho Pi Zero 2W
GPIO_AVAILABLE = False
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO = None
    print("⚠️  RPi.GPIO not available - GPIO control disabled")

def euler_from_quaternion(x, y, z, w):
    """Chuyển đổi Quaternion sang góc Euler (yaw)."""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

class WebBridgePi:
    """
    Phiên bản tối ưu cho Raspberry Pi Zero 2W - đã loại bỏ Wall Follower.
    """
    def __init__(self, node: Node):
        global GPIO_AVAILABLE

        self._node = node
        self._logger = node.get_logger()
        
        # Biến để lọc NaN
        self.last_valid_joint_positions = {}
        
        # Cấu hình cơ bản
        self.port = 8765
        self.host = "0.0.0.0"
        self.clients = set()
        self.server_loop = None
        
        # Throttling tối ưu
        self.last_map_time = 0
        self.last_odom_time = 0
        self.map_throttle = 2.0  # Tăng lên 2s để giảm tải
        self.odom_throttle = 0.5  # Tăng lên 0.5s
        
        # Lưu trữ dữ liệu
        self.last_map_data = None
        self.last_odom_data = None
        
        # GPIO Control
        self.motor_pin = 27
        self.motor_enabled = False
        self.motor_control_pin = 25
        self.motor_control_enabled = False
        
        # LiDAR PWM Control
        self.lidar_pwm_pin = 18
        self.lidar_pwm = None
        self.lidar_enabled = True
        self.lidar_duty_cycle = 7
        
        # GPIO13 PWM Control - Thêm mới
        self.gpio13_pwm_pin = 13
        self.gpio13_pwm = None
        self.gpio13_enabled = True  # Thay đổi từ False thành True
        self.gpio13_duty_cycle = 60  # Thay đổi từ 20 thành 60 (normal level)
        self.gpio13_pwm_levels = {
            'quiet': 20,      # Yên tĩnh - 20%
            'normal': 60,     # Bình thường - 60%
            'high': 100       # Hiệu suất cao - 100%
        }
        
        # Setup GPIO
        if GPIO_AVAILABLE:
            try:
                # Stop existing pwm.py process
                try:
                    subprocess.run(["pkill", "-f", "pwm.py"], capture_output=True, timeout=2)
                except:
                    pass
                
                GPIO.setmode(GPIO.BCM)
                
                # Setup motor pins
                GPIO.setup(self.motor_pin, GPIO.OUT)
                GPIO.output(self.motor_pin, GPIO.LOW)
                
                GPIO.setup(self.motor_control_pin, GPIO.OUT)
                GPIO.output(self.motor_control_pin, GPIO.LOW)
                
                # Setup LiDAR PWM
                GPIO.setup(self.lidar_pwm_pin, GPIO.OUT)
                self.lidar_pwm = GPIO.PWM(self.lidar_pwm_pin, 100)
                
                initial_duty = 0 if not self.lidar_enabled else self.lidar_duty_cycle
                self.lidar_pwm.start(initial_duty)
                
                # Setup GPIO13 PWM - Thêm mới
                GPIO.setup(self.gpio13_pwm_pin, GPIO.OUT)
                self.gpio13_pwm = GPIO.PWM(self.gpio13_pwm_pin, 100)
                self.gpio13_pwm.start(60)  # Bắt đầu với duty cycle 60% (normal level)
                
            except Exception as e:
                self._logger.error(f"❌ GPIO setup failed: {e}")
                GPIO_AVAILABLE = False
        else:
            self._logger.warning("⚠️  GPIO not available - motor and LiDAR control disabled")
        
        self._logger.info("🚀 Khởi tạo Web Bridge cho Pi Zero 2W...")

    def start(self):
        """Khởi động các thành phần của bridge."""
        try:
            # Tạo Subscribers
            self._node.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
            self._node.create_subscription(Odometry, '/odom', self.odom_callback, 10)
            self._node.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
            
            # Setup publishers
            self.cmd_vel_publisher = self._node.create_publisher(Twist, '/cmd_vel', 10)
            self.joint_states_sanitized_publisher = self._node.create_publisher(JointState, '/joint_states_sanitized', 10)
            
            # Khởi động WebSocket Server
            server_thread = threading.Thread(target=self.start_server_thread, daemon=True)
            server_thread.start()
            
            self._logger.info(f"✅ Web Bridge đã khởi động thành công")
        except Exception as e:
            self._logger.error(f"❌ Lỗi khởi động Web Bridge: {e}")
    
    def start_server_thread(self):
        """Hàm target cho thread của server."""
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
        """Logic chính của server."""
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
            # Gửi dữ liệu hiện tại
            if self.last_map_data or self.last_odom_data:
                initial_data = {
                    'map': self.last_map_data,
                    'odom': self.last_odom_data,
                    'timestamp': time.time()
                }
                await websocket.send(json.dumps(initial_data))

            # Lắng nghe messages
            try:
                async for message in websocket:
                    try:
                        data = json.loads(message)
                        if data.get('action') == 'ping':
                            await websocket.send(json.dumps({'type': 'pong', 'timestamp': time.time()}))
                        elif data.get('action') == 'control_lidar':
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
                            enabled = data.get('enabled', False)
                            pin = data.get('pin', 27)
                            success, message = self.control_motor(enabled, pin)
                            
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
                        elif data.get('action') == 'send_cmd_vel':
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
                        elif data.get('action') == 'control_gpio13_pwm':
                            level = data.get('level', 'quiet')
                            enabled = data.get('enabled', True)
                            success, message = self.control_gpio13_pwm(level, enabled)
                            
                            response = {
                                'type': 'control_response',
                                'action': 'control_gpio13_pwm',
                                'success': success,
                                'enabled': enabled if success else self.gpio13_enabled,
                                'level': level if success else self.get_gpio13_level_name(),
                                'duty_cycle': self.gpio13_duty_cycle,
                                'message': message,
                                'timestamp': time.time()
                            }
                            await websocket.send(json.dumps(response))
                    except json.JSONDecodeError:
                        pass
                    except Exception as e:
                        self._logger.warning(f"⚠️  Error processing message from {client_ip}: {e}")
            except websockets.exceptions.ConnectionClosed:
                pass
        except Exception as e:
            self._logger.warning(f"⚠️  Lỗi kết nối với client {client_ip}: {e}")
        finally:
            self.clients.discard(websocket)

    def map_callback(self, msg):
        """Xử lý dữ liệu bản đồ với throttling."""
        current_time = time.time()
        if current_time - self.last_map_time < self.map_throttle:
            return
        
        self.last_map_time = current_time
        
        try:
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
                'data': list(msg.data) if len(msg.data) < 50000 else None  # Giảm xuống 50k
            }
            
            if map_info['data'] is None:
                map_info['message'] = f"Map quá lớn ({len(msg.data)} cells > 50k), chỉ gửi metadata"
            
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
        
    def joint_states_callback(self, msg):
        """Lắng nghe /joint_states, lọc NaN và xuất bản ra topic đã được làm sạch."""
        sanitized_msg = JointState()
        sanitized_msg.header = msg.header
        sanitized_msg.name = msg.name
        sanitized_msg.velocity = msg.velocity
        sanitized_msg.effort = msg.effort
        
        new_positions = []
        is_dirty = False
        for i, name in enumerate(msg.name):
            pos = msg.position[i]
            if math.isnan(pos):
                is_dirty = True
                clean_pos = self.last_valid_joint_positions.get(name, 0.0)
                new_positions.append(clean_pos)
            else:
                self.last_valid_joint_positions[name] = pos
                new_positions.append(pos)
        
        if is_dirty:
            self._logger.warn("NaN detected in /joint_states, publishing sanitized version.")
        
        sanitized_msg.position = new_positions
        self.joint_states_sanitized_publisher.publish(sanitized_msg)

    def broadcast(self, message_data):
        """Gửi message tới tất cả các client."""
        if not self.clients:
            return
        
        try:
            message_json = json.dumps(message_data)
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
        
        for client in self.clients.copy():
            try:
                tasks.append(client.send(message_json))
            except (websockets.exceptions.ConnectionClosed, 
                    websockets.exceptions.ConnectionClosedError,
                    websockets.exceptions.ConnectionClosedOK,
                    AttributeError):
                disconnected_clients.add(client)
            except Exception:
                disconnected_clients.add(client)
        
        # Remove disconnected clients
        for client in disconnected_clients:
            self.clients.discard(client)
        
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

    # --- Control Methods ---
    def control_motor(self, enabled: bool, pin: int = 27):
        """Điều khiển motor qua GPIO pin (27 hoặc 25)."""
        if not GPIO_AVAILABLE:
            return False, "GPIO not available"
        
        try:
            if pin == 27:
                GPIO.output(self.motor_pin, GPIO.HIGH if enabled else GPIO.LOW)
                self.motor_enabled = enabled
                status = "HIGH" if enabled else "LOW"
                return True, f"GPIO{self.motor_pin} set to {status}"
            elif pin == 25:
                GPIO.output(self.motor_control_pin, GPIO.HIGH if enabled else GPIO.LOW)
                self.motor_control_enabled = enabled
                status = "HIGH" if enabled else "LOW"
                return True, f"GPIO{self.motor_control_pin} set to {status}"
            else:
                return False, f"Invalid pin {pin} - only 27 and 25 supported"
        except Exception as e:
            return False, str(e)
    
    def control_lidar(self, enabled: bool):
        """Điều khiển LiDAR rotation only."""
        if enabled == self.lidar_enabled:
            return True, f"LiDAR already {'enabled' if enabled else 'disabled'}"
        
        try:
            if enabled:
                if not GPIO_AVAILABLE or not self.lidar_pwm:
                    return False, "GPIO PWM not available"
                
                self.lidar_pwm.ChangeDutyCycle(self.lidar_duty_cycle)
                self.lidar_enabled = True
                return True, "LiDAR started successfully"
            else:
                if not GPIO_AVAILABLE or not self.lidar_pwm:
                    return False, "GPIO PWM not available"
                
                self.lidar_pwm.ChangeDutyCycle(0)
                self.lidar_enabled = False
                self._send_stop_command()
                return True, "LiDAR stopped"
                
        except Exception as e:
            return False, str(e)

    def _send_stop_command(self):
        """Gửi lệnh dừng robot để safety khi LiDAR tắt."""
        try:
            stop_pub = self._node.create_publisher(Twist, '/cmd_vel', 10)
            stop_msg = Twist()
            
            for i in range(5):  # Giảm xuống 5 lần
                stop_pub.publish(stop_msg)
                time.sleep(0.05)
                
            # Emergency fallback: disable motor power and control
            if GPIO_AVAILABLE:
                try:
                    if self.motor_enabled:
                        GPIO.output(self.motor_pin, GPIO.LOW)
                        self.motor_enabled = False
                    
                    if self.motor_control_enabled:
                        GPIO.output(self.motor_control_pin, GPIO.LOW)
                        self.motor_control_enabled = False
                except:
                    pass
                    
        except Exception as e:
            self._logger.error(f"❌ Failed to send stop command: {e}")

    def publish_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        """Publish velocity command for manual robot control."""
        try:
            # Safety limits
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
            
            return True, "Velocity command sent successfully"
            
        except Exception as e:
            return False, f"Failed to send velocity command: {e}"

    def control_gpio13_pwm(self, level='quiet', enabled=True):
        """Điều khiển PWM GPIO13 với 3 mức độ."""
        if not GPIO_AVAILABLE:
            return False, "GPIO not available"
        
        try:
            if level not in self.gpio13_pwm_levels:
                return False, f"Invalid level '{level}' - use 'quiet', 'normal', or 'high'"
            
            duty_cycle = self.gpio13_pwm_levels[level]
            
            if enabled:
                if not self.gpio13_pwm:
                    return False, "GPIO13 PWM not available"
                
                self.gpio13_pwm.ChangeDutyCycle(duty_cycle)
                self.gpio13_enabled = True
                self.gpio13_duty_cycle = duty_cycle
                return True, f"GPIO13 PWM enabled at {level} level ({duty_cycle}%)"
            else:
                if not self.gpio13_pwm:
                    return False, "GPIO13 PWM not available"
                
                self.gpio13_pwm.ChangeDutyCycle(0)
                self.gpio13_enabled = False
                self.gpio13_duty_cycle = 0
                return True, "GPIO13 PWM disabled"
                
        except Exception as e:
            return False, str(e)

    def get_gpio13_level_name(self):
        """Lấy tên level hiện tại của GPIO13 PWM."""
        for level_name, duty_cycle in self.gpio13_pwm_levels.items():
            if duty_cycle == self.gpio13_duty_cycle:
                return level_name
        return 'unknown'

    def cleanup(self):
        """Cleanup GPIO và processes khi shutdown."""
        try:
            if GPIO_AVAILABLE:
                # Turn off motor power
                GPIO.output(self.motor_pin, GPIO.LOW)
                
                # Turn off motor control
                GPIO.output(self.motor_control_pin, GPIO.LOW)
                
                # Stop LiDAR PWM
                if self.lidar_pwm:
                    self.lidar_pwm.ChangeDutyCycle(0)
                    self.lidar_pwm.stop()
                
                # Stop GPIO13 PWM - Thêm mới
                if self.gpio13_pwm:
                    self.gpio13_pwm.ChangeDutyCycle(0)
                    self.gpio13_pwm.stop()
                
                GPIO.cleanup()
            
            self._logger.info("✅ Cleanup completed")
            
        except Exception as e:
            self._logger.error(f"❌ Cleanup error: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    node = rclpy.create_node('web_bridge_pi_node')
    bridge = WebBridgePi(node)
    bridge.start()
    
    try:
        node.get_logger().info("🔄 Web Bridge Pi đang chạy...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] Tắt cầu nối Web...")
    finally:
        bridge.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
