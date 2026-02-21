import time
from threading import Condition
import io

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

try:
    from PIL import Image as PILImage
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

try:
    import cv2
    import numpy as np
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

class Camera:
    def __init__(
        self,
        node,
        input_topic: str = "/image_raw",
        preview_size: tuple = (640, 480),
        hflip: bool = False,
        vflip: bool = False,
        stream_size: tuple = (400, 300),
    ):
        self._node = node
        self._input_topic = input_topic
        self._preview_size = preview_size
        self._hflip = hflip
        self._vflip = vflip
        self._stream_size = stream_size
        self._condition = Condition()
        self._frame = None
        self._format = None
        self._subscription = None
        self.streaming = False
        self._capture = None
        self._capture_thread = None
        self._stop_capture_event = None

        if not HAS_PIL and not HAS_CV2:
            self._node.get_logger().error("Neither PIL nor OpenCV found. Camera compression will fail.")

    def _compress_cv2_image(self, img) -> tuple:
        try:
            if self._hflip:
                img = cv2.flip(img, 1)
            if self._vflip:
                img = cv2.flip(img, 0)
            
            if self._stream_size:
                img = cv2.resize(img, self._stream_size)

            # Ensure we are encoding a valid BGR image to JPEG
            _, compressed = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            return compressed.tobytes(), "jpeg"
        except Exception as e:
            self._node.get_logger().error(f"OpenCV compression failed: {e}")
            return None, None

    def _capture_loop(self):
        self._node.get_logger().info("Starting OpenCV VideoCapture(0)...")
        # Try different backends
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not cap.isOpened():
             self._node.get_logger().warn("Failed to open /dev/video0 with V4L2, trying default backend...")
             cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
             self._node.get_logger().error("Failed to open /dev/video0")
             return

        # Set resolution if possible
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._preview_size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._preview_size[1])
        
        frame_count = 0
        while not self._stop_capture_event.is_set():
            ret, frame = cap.read()
            if not ret:
                if frame_count % 100 == 0:
                    self._node.get_logger().warn("VideoCapture read failed (no frame)")
                time.sleep(0.1)
                continue
            
            frame_count += 1
            if frame_count % 300 == 0:
                 self._node.get_logger().info(f"Captured {frame_count} frames successfully")

            compressed_data, fmt = self._compress_cv2_image(frame)
            if compressed_data:
                with self._condition:
                    self._frame = compressed_data
                    self._format = fmt
                    self._condition.notify_all()
            time.sleep(0.03) # Limit to ~30fps
        
        cap.release()
        self._node.get_logger().info("OpenCV VideoCapture stopped")

    def _compress_image(self, msg: Image) -> tuple:
        # Returns (bytes, format)
        if HAS_PIL:
            try:
                # Assuming rgb8 based on user input
                mode = 'RGB'
                if msg.encoding == 'mono8':
                    mode = 'L'
                elif msg.encoding == 'rgba8':
                    mode = 'RGBA'
                
                img = PILImage.frombytes(mode, (msg.width, msg.height), bytes(msg.data))
                
                if self._hflip:
                    img = img.transpose(PILImage.FLIP_LEFT_RIGHT)
                if self._vflip:
                    img = img.transpose(PILImage.FLIP_TOP_BOTTOM)
                
                if self._stream_size and (msg.width != self._stream_size[0] or msg.height != self._stream_size[1]):
                    img = img.resize(self._stream_size, PILImage.NEAREST)

                buffer = io.BytesIO()
                img.save(buffer, format="JPEG", quality=80)
                return buffer.getvalue(), "jpeg"
            except Exception as e:
                self._node.get_logger().error(f"PIL compression failed: {e}")
                return None, None

        elif HAS_CV2:
            try:
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
                
                # Convert to BGR for OpenCV if needed
                if msg.encoding == 'rgb8':
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                elif msg.encoding == 'rgba8':
                    img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
                
                if self._hflip:
                    img = cv2.flip(img, 1)
                if self._vflip:
                    img = cv2.flip(img, 0)
                
                if self._stream_size and (msg.width != self._stream_size[0] or msg.height != self._stream_size[1]):
                    img = cv2.resize(img, self._stream_size)

                _, compressed = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                return compressed.tobytes(), "jpeg"
            except Exception as e:
                self._node.get_logger().error(f"OpenCV compression failed: {e}")
                return None, None
        
        return None, None

    def _on_raw_image(self, msg: Image) -> None:
        self._node.get_logger().info(f"Received raw image: {msg.width}x{msg.height}, encoding={msg.encoding}")
        compressed_data, fmt = self._compress_image(msg)
        if compressed_data:
            self._node.get_logger().info(f"Compression success: {len(compressed_data)} bytes")
            with self._condition:
                self._frame = compressed_data
                self._format = fmt
                self._condition.notify_all()
        else:
            self._node.get_logger().warn("Compression failed")

    def _on_compressed_image(self, msg: CompressedImage) -> None:
        with self._condition:
            self._frame = bytes(msg.data)
            self._format = msg.format
            self._condition.notify_all()

    def start_image(self) -> None:
        self.start_stream()

    def save_image(self, filename: str) -> dict:
        if not self.streaming:
            self.start_stream()
        frame = self.get_frame()
        if frame is None:
            return None
        try:
            with open(filename, "wb") as output_file:
                output_file.write(frame)
            return {"format": self._format}
        except Exception as e:
            print(f"Error capturing image: {e}")
            return None

    def start_stream(self, filename: str = None) -> None:
        if not self.streaming:
            if filename is not None:
                raise NotImplementedError("Recording to file is not supported with camera_ros input.")
            
            # If input_topic is empty or "0", use hardware capture
            if not self._input_topic or self._input_topic == "0" or self._input_topic == "/0":
                 import threading
                 self._stop_capture_event = threading.Event()
                 self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
                 self._capture_thread.start()
            else:
                qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
                
                # Decide based on topic name or just try both?
                # Since user has /image_raw, we prefer Raw Image subscription
                # But let's check if the topic string contains "compressed"
                
                if "compressed" in self._input_topic:
                     self._subscription = self._node.create_subscription(
                        CompressedImage, self._input_topic, self._on_compressed_image, qos_profile
                    )
                else:
                    self._subscription = self._node.create_subscription(
                        Image, self._input_topic, self._on_raw_image, qos_profile
                    )
            
            self.streaming = True

    def stop_stream(self) -> None:
        if self.streaming:
            try:
                if self._capture_thread:
                    self._stop_capture_event.set()
                    self._capture_thread.join(timeout=2.0)
                    self._capture_thread = None
                    self._stop_capture_event = None

                if self._subscription is not None:
                    self._node.destroy_subscription(self._subscription)
                self._subscription = None
                self.streaming = False
            except Exception as e:
                print(f"Error stopping stream: {e}")

    def get_frame(self) -> bytes:
        with self._condition:
            while self._frame is None:
                self._condition.wait()
            return self._frame

    def get_format(self) -> str:
        return self._format

    def save_video(self, filename: str, duration: int = 10) -> None:
        raise NotImplementedError(
            f"save_video({filename}, {duration}) is not supported with camera_ros input."
        )

    def close(self) -> None:
        if self.streaming:
            self.stop_stream()

if __name__ == "__main__":
    rclpy.init()
    node = rclpy.create_node("camera_driver_test")
    camera = Camera(node=node)
    camera.start_stream()
    time.sleep(1)
    camera.save_image(filename="image.jpg")
    camera.close()
    node.destroy_node()
    rclpy.shutdown()
