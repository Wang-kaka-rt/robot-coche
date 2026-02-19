import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from freenove_4wd_nodes.camera_driver import Camera


class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__("freenove_camera")

        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("topic", "/image_raw/compressed")
        self.declare_parameter("input_topic", "/camera/image_raw/compressed")
        self.declare_parameter("stream_width", 400)
        self.declare_parameter("stream_height", 300)
        self.declare_parameter("hflip", False)
        self.declare_parameter("vflip", False)
        self.declare_parameter("publish_min_interval_s", 0.0)

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._topic = str(self.get_parameter("topic").value)
        self._input_topic = str(self.get_parameter("input_topic").value)
        self._stream_width = int(self.get_parameter("stream_width").value)
        self._stream_height = int(self.get_parameter("stream_height").value)
        self._hflip = bool(self.get_parameter("hflip").value)
        self._vflip = bool(self.get_parameter("vflip").value)
        self._publish_min_interval_s = float(self.get_parameter("publish_min_interval_s").value)

        self._pub = self.create_publisher(CompressedImage, self._topic, 10)

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        return super().destroy_node()

    def _run(self) -> None:
        camera = None
        try:
            camera = Camera(
                node=self,
                input_topic=self._input_topic,
                stream_size=(self._stream_width, self._stream_height),
                hflip=self._hflip,
                vflip=self._vflip,
            )
            camera.start_stream()
            last_pub = 0.0
            while rclpy.ok() and (not self._stop_event.is_set()):
                frame = camera.get_frame()
                if frame is None:
                    continue
                now = time.time()
                if self._publish_min_interval_s > 0.0 and (now - last_pub) < self._publish_min_interval_s:
                    continue
                last_pub = now

                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self._frame_id
                msg.format = camera.get_format() or "jpeg"
                msg.data = frame
                self._pub.publish(msg)
        except Exception:
            pass
        finally:
            if camera is not None:
                try:
                    camera.stop_stream()
                except Exception:
                    pass
                try:
                    camera.close()
                except Exception:
                    pass


def main() -> None:
    rclpy.init()
    node = CameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
