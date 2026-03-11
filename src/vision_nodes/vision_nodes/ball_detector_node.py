"""
Ball Detector Node for ROS2.

This node subscribes to a camera topic, detects red transport cards using HSV color segmentation,
and publishes detection results as custom messages and visualization markers.

Designed to be resource-efficient for Raspberry Pi 4.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray, Bool
import cv2
from cv_bridge import CvBridge
import numpy as np
from typing import Optional

from vision_nodes.vision_utils import BallDetector


class BallDetectorNode(Node):
    """
    ROS2 node for detecting white balls in video stream.
    
    Subscriptions:
        - /image_raw (sensor_msgs/Image): Raw camera feed
        - /image_raw/compressed (sensor_msgs/CompressedImage): Compressed camera feed
        
    Publications:
        - /ball_detection/position (geometry_msgs/Point): Ball center position [x, y, 0]
        - /ball_detection/radius (std_msgs/Float32MultiArray): [radius_px, distance_est]
        - /ball_detection/status (std_msgs/Bool): True if ball detected
        - /ball_detection/debug_image (sensor_msgs/CompressedImage): Debug visualization
    """
    
    def __init__(self):
        super().__init__('ball_detector_node')
        
        # Parameters
        self.declare_parameter('camera_topic', '/image_raw/compressed')
        self.declare_parameter('min_ball_area', 500)
        self.declare_parameter('max_ball_area', 100000)
        self.declare_parameter('publish_debug_images', False)
        self.declare_parameter('debug_image_quality', 80)
        self.declare_parameter('detection_threshold_area', 50)
        
        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        min_area = self.get_parameter('min_ball_area').value
        max_area = self.get_parameter('max_ball_area').value
        self.publish_debug = self.get_parameter('publish_debug_images').value
        self.debug_quality = self.get_parameter('debug_image_quality').value
        self.det_threshold = self.get_parameter('detection_threshold_area').value
        
        # Initialize detector
        self.detector = BallDetector(min_area=min_area, max_area=max_area)
        self.bridge = CvBridge()
        
        # Publishers
        self.pub_position = self.create_publisher(
            Point, '/ball_detection/position', 10)
        self.pub_detection = self.create_publisher(
            Float32MultiArray, '/ball_detection/radius', 10)
        self.pub_status = self.create_publisher(
            Bool, '/ball_detection/status', 10)
        
        if self.publish_debug:
            self.pub_debug = self.create_publisher(
                CompressedImage, '/ball_detection/debug_image', 
                QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
        
        # Subscription
        # Try compressed first (faster)
        if 'compressed' in self.camera_topic:
            self.create_subscription(
                CompressedImage, self.camera_topic,
                self._on_image_compressed, 10)
            self.get_logger().info(f'Subscribed to {self.camera_topic} (compressed)')
        else:
            self.create_subscription(
                Image, self.camera_topic,
                self._on_image_raw, 10)
            self.get_logger().info(f'Subscribed to {self.camera_topic} (raw)')
        
        self.get_logger().info('Ball Detector Node initialized')
        self.frame_count = 0
        self.detection_count = 0
    
    def _on_image_compressed(self, msg: CompressedImage) -> None:
        """Handle compressed image from camera."""
        try:
            # Decode compressed image
            nparr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if frame is None:
                self.get_logger().warn('Failed to decode compressed image')
                return

            # libcamera RGB888 is actually BGR in memory; the camera node
            # then does an extra RGB→BGR swap, so the JPEG ends up with
            # R and B channels swapped.  Undo that here.
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            self._process_frame(frame)
        except Exception as e:
            self.get_logger().error(f'Error processing compressed image: {e}')
    
    def _on_image_raw(self, msg: Image) -> None:
        """Handle raw image from camera."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._process_frame(frame)
        except Exception as e:
            self.get_logger().error(f'Error processing raw image: {e}')
    
    def _process_frame(self, frame: np.ndarray) -> None:
        """
        Process a single frame for ball detection.
        
        Args:
            frame: OpenCV image (BGR)
        """
        self.frame_count += 1
        
        # Detect card
        detection = self.detector.detect(frame)
        
        # Publish status
        status_msg = Bool()
        status_msg.data = detection is not None
        self.pub_status.publish(status_msg)
        
        if detection is not None:
            x, y, w, h = detection
            
            # Publish position (center of card)
            pos_msg = Point()
            pos_msg.x = float(x + w // 2)
            pos_msg.y = float(y + h // 2)
            pos_msg.z = 0.0
            self.pub_position.publish(pos_msg)
            
            # Publish size and estimated distance
            # Size estimate: average of width/height
            size = (w + h) / 2.0
            det_msg = Float32MultiArray()
            det_msg.data = [float(size), float(size) / 25.0]  # Simple distance estimate
            self.pub_detection.publish(det_msg)
            
            self.detection_count += 1
            
            # Debug logging (every 30 frames)
            if self.frame_count % 30 == 0:
                self.get_logger().debug(
                    f'Card detected: center=({x + w // 2}, {y + h // 2}), size={w}x{h}')
        
        # Publish debug image if requested
        if self.publish_debug:
            debug_frame = self.detector.draw_detection(frame, detection)
            self._publish_debug_image(debug_frame)
    
    def _publish_debug_image(self, frame: np.ndarray) -> None:
        """Publish debug visualization image."""
        try:
            # Encode to JPEG
            ok, jpeg_data = cv2.imencode('.jpg', frame,
                                        [cv2.IMWRITE_JPEG_QUALITY, self.debug_quality])
            if not ok:
                return
            
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = 'jpeg'
            msg.data = jpeg_data.tobytes()
            self.pub_debug.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing debug image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = BallDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Ball Detector: processed {node.frame_count} frames, '
            f'detected ball in {node.detection_count} frames')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
