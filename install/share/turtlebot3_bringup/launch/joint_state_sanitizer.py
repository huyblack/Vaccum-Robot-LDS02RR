#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointStateSanitizer(Node):
    def __init__(self):
        super().__init__('joint_state_sanitizer')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(JointState, '/joint_states_sanitized', 10)
        
        # Lưu trữ vị trí hợp lệ cuối cùng của mỗi khớp
        self.last_valid_positions = {}
        
        self.get_logger().info('Joint State Sanitizer is running and cleaning /joint_states.')

    def listener_callback(self, msg):
        sanitized_msg = JointState()
        sanitized_msg.header = msg.header
        sanitized_msg.name = msg.name
        sanitized_msg.velocity = msg.velocity
        sanitized_msg.effort = msg.effort
        
        new_positions = []
        for i, name in enumerate(msg.name):
            pos = msg.position[i]
            # Kiểm tra xem vị trí có phải là NaN không
            if math.isnan(pos):
                # Nếu là NaN, sử dụng vị trí hợp lệ cuối cùng đã lưu
                # Nếu chưa có, mặc định là 0.0
                clean_pos = self.last_valid_positions.get(name, 0.0)
                new_positions.append(clean_pos)
                self.get_logger().warn(f"NaN detected for joint '{name}'. Replacing with '{clean_pos}'.")
            else:
                # Nếu hợp lệ, cập nhật giá trị đã lưu và sử dụng nó
                self.last_valid_positions[name] = pos
                new_positions.append(pos)
        
        sanitized_msg.position = new_positions
        self.publisher.publish(sanitized_msg)

def main(args=None):
    rclpy.init(args=args)
    sanitizer = JointStateSanitizer()
    rclpy.spin(sanitizer)
    sanitizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 