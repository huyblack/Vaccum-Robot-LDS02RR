#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
import asyncio
import websockets
import json
import threading
import math
import time

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
    Phiên bản tối ưu cho Raspberry Pi với throttling và error handling.
    """
    def __init__(self, node: Node):
        self._node = node
        self._logger = node.get_logger()
        
        # --- Cấu hình tối ưu cho Pi ---
        self.port = 8765
        self.host = "0.0.0.0"  # Cho phép kết nối từ mạng
        self.clients = set()
        self.server_loop = None
        
        # --- Throttling để tiết kiệm tài nguyên ---
        self.last_map_time = 0
        self.last_odom_time = 0
        self.map_throttle = 2.0  # Giây - Map update ít hơn
        self.odom_throttle = 0.5  # Giây - Odom update thường xuyên hơn
        
        # --- Lưu trữ dữ liệu ---
        self.last_map_data = None
        self.last_odom_data = None
        
        self._logger.info("🚀 Khởi tạo Web Bridge cho Pi...")

    def start(self):
        """Khởi động các thành phần của bridge."""
        try:
            # --- Tạo Subscribers ---
            self._node.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
            self._node.create_subscription(Odometry, '/odom', self.odom_callback, 10)
            
            # --- Khởi động WebSocket Server trong một thread riêng ---
            server_thread = threading.Thread(target=self.start_server_thread, daemon=True)
            server_thread.start()
            
            self._logger.info(f"✅ Web Bridge đã khởi động thành công")
        except Exception as e:
            self._logger.error(f"❌ Lỗi khởi động Web Bridge: {e}")

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
            if self.last_map_data or self.last_odom_data:
                initial_data = {
                    'map': self.last_map_data,
                    'odom': self.last_odom_data,
                    'timestamp': time.time()
                }
                await websocket.send(json.dumps(initial_data))
                self._logger.info(f"📤 Đã gửi dữ liệu ban đầu cho {client_ip}")

            # Giữ kết nối mở
            await websocket.wait_closed()
        except websockets.exceptions.ConnectionClosed:
            self._logger.info(f"ℹ️  Client {client_ip} đã ngắt kết nối")
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
        self._logger.info("🗺️  Đã nhận dữ liệu Map mới")
        
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
                'data': list(msg.data) if len(msg.data) < 10000 else None  # Giới hạn data size
            }
            
            if map_info['data'] is None:
                map_info['message'] = f"Map quá lớn ({len(msg.data)} cells), chỉ gửi metadata"
            
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
                        'y': float(msg.pose.pose.position.y)
                    },
                    'orientation_yaw': float(yaw)
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
            
        tasks = []
        for client in self.clients.copy():  # Copy để tránh modification during iteration
            try:
                tasks.append(client.send(message_json))
            except Exception as e:
                self._logger.warning(f"⚠️  Lỗi chuẩn bị gửi đến client: {e}")
                self.clients.discard(client)
        
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 