#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
import asyncio
import websockets
import json
import threading
import math

def euler_from_quaternion(x, y, z, w):
    """
    Chuyển đổi Quaternion (dữ liệu định hướng của ROS) sang góc Euler (yaw).
    """
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

class WebBridge:
    """
    Lớp này quản lý logic của WebSocket server và giao tiếp với ROS.
    Nó không kế thừa từ Node để tránh lỗi __slots__.
    """
    def __init__(self, node: Node):
        self._node = node
        self._logger = node.get_logger()
        
        # --- Cấu hình ---
        self.port = 8765
        self.clients = set()
        self.server_loop = None  # Sẽ lưu event loop của server thread
        
        # --- Lưu trữ dữ liệu ---
        self.last_map_data = None
        self.last_odom_data = None
        
        self._logger.info("🚀 Khởi tạo Web Bridge...")

    def start(self):
        """Khởi động các thành phần của bridge."""
        # --- Tạo Subscribers ---
        self._node.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self._node.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # --- Khởi động WebSocket Server trong một thread riêng ---
        server_thread = threading.Thread(target=self.start_server_thread, daemon=True)
        server_thread.start()

    def start_server_thread(self):
        """Hàm target cho thread của server, thiết lập và chạy event loop."""
        self.server_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.server_loop)
        
        self.server_loop.run_until_complete(self.server_main())
        self.server_loop.close()

    async def server_main(self):
        """Logic chính của server, chạy vô tận."""
        self._logger.info(f"🛰️  WebSocket Server đang lắng nghe trên cổng {self.port}")
        # async with sẽ tự động quản lý việc khởi động và tắt server
        async with websockets.serve(self.handler, "0.0.0.0", self.port):
            await asyncio.Future()  # Chạy vô tận

    async def handler(self, websocket):
        """Xử lý các kết nối mới từ client."""
        self._logger.info(f"✅ Client mới đã kết nối từ {websocket.remote_address}")
        self.clients.add(websocket)
        try:
            # Gửi dữ liệu hiện tại ngay khi client kết nối
            if self.last_map_data or self.last_odom_data:
                initial_data = {
                    'map': self.last_map_data,
                    'odom': self.last_odom_data
                }
                if initial_data['map'] or initial_data['odom']:
                    await websocket.send(json.dumps(initial_data))

            # Giữ kết nối mở
            await websocket.wait_closed()
        except websockets.exceptions.ConnectionClosed:
            self._logger.info(f"ℹ️  Client {websocket.remote_address} đã ngắt kết nối.")
        finally:
            self.clients.remove(websocket)

    def map_callback(self, msg):
        """Xử lý dữ liệu bản đồ."""
        self._logger.info("🗺️  Đã nhận dữ liệu Map mới", throttle_duration_sec=5)
        
        map_info = {
            'type': 'map',
            'info': {
                'resolution': msg.info.resolution,
                'width': msg.info.width,
                'height': msg.info.height,
                'origin': {'x': msg.info.origin.position.x, 'y': msg.info.origin.position.y},
            },
            'data': list(msg.data)
        }
        self.last_map_data = map_info
        self.broadcast(map_info)
        
    def odom_callback(self, msg):
        """Xử lý dữ liệu odometry."""
        self._logger.info("🤖 Đã nhận dữ liệu Odometry", throttle_duration_sec=2)
        
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)

        odom_info = {
            'type': 'odom',
            'pose': {
                'position': {'x': msg.pose.pose.position.x, 'y': msg.pose.pose.position.y},
                'orientation_yaw': yaw
            }
        }
        self.last_odom_data = odom_info
        self.broadcast(odom_info)
        
    def broadcast(self, message_data):
        """Gửi message tới tất cả các client."""
        if not self.clients:
            return
            
        message_json = json.dumps(message_data)
        
        # Đảm bảo rằng coroutine được chạy trên đúng event loop của server
        if self.server_loop and self.server_loop.is_running():
            asyncio.run_coroutine_threadsafe(self.send_to_all(message_json), self.server_loop)

    async def send_to_all(self, message_json):
        """Hàm async để gửi dữ liệu."""
        if self.clients:
            tasks = [client.send(message_json) for client in self.clients]
            await asyncio.gather(*tasks, return_exceptions=True)

def main(args=None):
    rclpy.init(args=args)
    
    # Tạo node ROS2
    node = rclpy.create_node('web_bridge_node')
    
    # Tạo và khởi động bridge
    bridge = WebBridge(node)
    bridge.start()
    
    try:
        # Giữ cho node ROS hoạt động
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[INFO] Tắt cầu nối Web...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 