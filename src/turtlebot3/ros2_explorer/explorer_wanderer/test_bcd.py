#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Script test cho thuật toán BCD Coverage
Kiểm tra các chức năng của BCD algorithm
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import numpy as np
import time

class BCDTester(Node):
    """
    Tester cho thuật toán BCD Coverage
    """
    
    def __init__(self):
        super().__init__('bcd_tester')
        
        # Subscribers
        self.map_subscription = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10
        )
        
        # Test variables
        self.test_map_received = False
        self.test_bcd_executed = False
        self.test_coverage_path_generated = False
        
        # Timer cho test
        self.test_timer = self.create_timer(2.0, self.run_tests)
        
        self.get_logger().info("BCD Tester initialized")
    
    def map_callback(self, msg):
        """
        Callback nhận bản đồ để test
        """
        self.test_map_received = True
        self.get_logger().info("Test: Map received successfully")
        
        # Test map properties
        self.test_map_properties(msg)
    
    def test_map_properties(self, map_msg):
        """
        Test các thuộc tính của bản đồ
        """
        try:
            # Kiểm tra resolution
            assert map_msg.info.resolution > 0, "Map resolution must be positive"
            
            # Kiểm tra kích thước
            assert map_msg.info.width > 0, "Map width must be positive"
            assert map_msg.info.height > 0, "Map height must be positive"
            
            # Kiểm tra dữ liệu
            assert len(map_msg.data) > 0, "Map data must not be empty"
            
            self.get_logger().info("Test: Map properties are valid")
            
        except AssertionError as e:
            self.get_logger().error(f"Test failed: {str(e)}")
    
    def run_tests(self):
        """
        Chạy các test
        """
        self.get_logger().info("Running BCD tests...")
        
        # Test 1: Kiểm tra map
        if self.test_map_received:
            self.get_logger().info("✓ Test 1 PASSED: Map received")
        else:
            self.get_logger().warn("✗ Test 1 FAILED: No map received")
        
        # Test 2: Kiểm tra BCD coverage node
        try:
            # Kiểm tra xem node có tồn tại không
            node_names = self.get_node_names()
            bcd_nodes = [name for name in node_names if 'bcd' in name.lower()]
            
            if bcd_nodes:
                self.get_logger().info(f"✓ Test 2 PASSED: BCD nodes found: {bcd_nodes}")
                self.test_bcd_executed = True
            else:
                self.get_logger().warn("✗ Test 2 FAILED: No BCD nodes found")
                
        except Exception as e:
            self.get_logger().error(f"Test 2 error: {str(e)}")
        
        # Test 3: Kiểm tra topics
        try:
            topics = self.get_topic_names_and_types()
            bcd_topics = [topic[0] for topic in topics if 'bcd' in topic[0].lower()]
            
            if bcd_topics:
                self.get_logger().info(f"✓ Test 3 PASSED: BCD topics found: {bcd_topics}")
            else:
                self.get_logger().warn("✗ Test 3 FAILED: No BCD topics found")
                
        except Exception as e:
            self.get_logger().error(f"Test 3 error: {str(e)}")
        
        # Test 4: Kiểm tra services
        try:
            services = self.get_service_names_and_types()
            bcd_services = [service[0] for service in services if 'bcd' in service[0].lower()]
            
            if bcd_services:
                self.get_logger().info(f"✓ Test 4 PASSED: BCD services found: {bcd_services}")
            else:
                self.get_logger().info("Test 4: No BCD services found (this is normal)")
                
        except Exception as e:
            self.get_logger().error(f"Test 4 error: {str(e)}")
        
        # Test 5: Performance test
        self.test_performance()
        
        # Summary
        self.print_test_summary()
    
    def test_performance(self):
        """
        Test hiệu suất của hệ thống
        """
        try:
            # Kiểm tra memory usage
            import psutil
            process = psutil.Process()
            memory_mb = process.memory_info().rss / 1024 / 1024
            
            if memory_mb < 500:  # Dưới 500MB
                self.get_logger().info(f"✓ Test 5 PASSED: Memory usage: {memory_mb:.1f}MB")
            else:
                self.get_logger().warn(f"✗ Test 5 WARNING: High memory usage: {memory_mb:.1f}MB")
                
        except ImportError:
            self.get_logger().info("Test 5: psutil not available, skipping memory test")
        except Exception as e:
            self.get_logger().error(f"Test 5 error: {str(e)}")
    
    def print_test_summary(self):
        """
        In tổng kết test
        """
        self.get_logger().info("=" * 50)
        self.get_logger().info("BCD TEST SUMMARY")
        self.get_logger().info("=" * 50)
        
        tests = [
            ("Map Reception", self.test_map_received),
            ("BCD Execution", self.test_bcd_executed),
            ("Coverage Path", self.test_coverage_path_generated)
        ]
        
        passed = 0
        total = len(tests)
        
        for test_name, result in tests:
            status = "PASSED" if result else "FAILED"
            self.get_logger().info(f"{test_name}: {status}")
            if result:
                passed += 1
        
        self.get_logger().info(f"Overall: {passed}/{total} tests passed")
        
        if passed == total:
            self.get_logger().info("🎉 All tests PASSED! BCD system is working correctly.")
        else:
            self.get_logger().warn("⚠️  Some tests FAILED. Please check the system.")
        
        self.get_logger().info("=" * 50)


def main(args=None):
    rclpy.init(args=args)
    
    tester = BCDTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 