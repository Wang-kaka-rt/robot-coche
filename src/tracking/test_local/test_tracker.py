"""
Test script for object tracking functionality.

This script tests the tracking node without requiring the full robot hardware.
Useful for development and debugging.

Usage:
  python test_local/test_tracker.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
from geometry_msgs.msg import Point
import time
import threading


class TrackerTester(Node):
    """Test node to simulate object detection for tracker testing."""
    
    def __init__(self):
        super().__init__('tracker_tester')
        
        # Publishers to simulate detection
        self.pub_status = self.create_publisher(Bool, '/ball_detection/status', 10)
        self.pub_position = self.create_publisher(Point, '/ball_detection/position', 10)
        self.pub_size = self.create_publisher(Float32MultiArray, '/ball_detection/radius', 10)
        
        # Subscriber to monitor tracker output
        self.create_subscription(Point, '/cmd_vel', self._on_cmd_vel, 10)
        
        self.test_running = True
        self.test_phase = 0
        self.test_start_time = time.time()
        
        self.get_logger().info('Tracker Tester initialized')
        self.get_logger().info('Starting test sequence...')
    
    def _on_cmd_vel(self, msg):
        """Log velocity commands from tracker."""
        self.get_logger().info(
            f'cmd_vel: linear={msg.linear.x:.3f}, angular={msg.angular.z:.3f}'
        )
    
    def run_test_sequence(self):
        """Run through different test phases."""
        
        while rclpy.ok() and self.test_running:
            now = time.time()
            elapsed = now - self.test_start_time
            
            # Phase 1: No detection (0-3 seconds)
            if elapsed < 3.0:
                self._publish_detection(False, None, None)
                if elapsed % 1.0 < 0.1:
                    self.get_logger().info('Phase 1: No detection')
            
            # Phase 2: Object appears on right (3-6 seconds)
            elif elapsed < 6.0:
                position = Point(x=300.0, y=150.0, z=0.0)  # Right side
                size = 50.0
                self._publish_detection(True, position, size)
                if elapsed % 1.0 < 0.1:
                    self.get_logger().info('Phase 2: Object on right')
            
            # Phase 3: Object centered (6-9 seconds)
            elif elapsed < 9.0:
                position = Point(x=200.0, y=150.0, z=0.0)  # Center
                size = 60.0
                self._publish_detection(True, position, size)
                if elapsed % 1.0 < 0.1:
                    self.get_logger().info('Phase 3: Object centered - should follow')
            
            # Phase 4: Object too close (9-12 seconds)
            elif elapsed < 12.0:
                position = Point(x=200.0, y=150.0, z=0.0)
                size = 120.0  # Too large
                self._publish_detection(True, position, size)
                if elapsed % 1.0 < 0.1:
                    self.get_logger().info('Phase 4: Object too close - should back up')
            
            # Phase 5: Object lost (12-15 seconds)
            elif elapsed < 15.0:
                self._publish_detection(False, None, None)
                if elapsed % 1.0 < 0.1:
                    self.get_logger().info('Phase 5: Object lost - should search')
            
            # Restart test
            else:
                self.test_start_time = time.time()
                self.get_logger().info('=== Restarting test sequence ===')
            
            time.sleep(0.1)
    
    def _publish_detection(self, detected: bool, 
                           position: Point, size: float):
        """Publish detection data."""
        # Status
        status_msg = Bool()
        status_msg.data = detected
        self.pub_status.publish(status_msg)
        
        # Position
        if position:
            self.pub_position.publish(position)
        
        # Size
        if size is not None:
            size_msg = Float32MultiArray()
            size_msg.data = [size]
            self.pub_size.publish(size_msg)
    
    def shutdown(self):
        self.test_running = False


def main(args=None):
    rclpy.init(args=args)
    tester = TrackerTester()
    
    # Run test in separate thread
    test_thread = threading.Thread(target=tester.run_test_sequence)
    test_thread.start()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.shutdown()
        test_thread.join()
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
