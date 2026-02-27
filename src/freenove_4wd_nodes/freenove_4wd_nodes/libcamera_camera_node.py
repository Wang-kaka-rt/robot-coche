import os
import selectors
import sys
import threading
import time
import traceback
from typing import Optional
 
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
 
 
def _import_libcamera():
    try:
        import libcamera  # type: ignore
        if hasattr(libcamera, "CameraManager"):
            return libcamera
    except Exception:
        pass
 
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    py_path = os.path.join(repo_root, "libcamera", "build", "src", "py")
    if os.path.isdir(py_path):
        sys.path.insert(0, py_path)
    if "libcamera" in sys.modules:
        del sys.modules["libcamera"]
    import libcamera  # type: ignore
    return libcamera
 
 
class _LibcameraCapture:
    def __init__(
        self,
        capture_width: int,
        capture_height: int,
        camera_index: int,
        prefer_pixel_format: str,
    ) -> None:
        self._libcam = _import_libcamera()
        self._MappedFrameBuffer = self._import_mfb()
 
        cm = self._libcam.CameraManager.singleton()
        cameras = list(cm.cameras)
        if not cameras:
            raise RuntimeError("No libcamera cameras found")
        if camera_index < 0 or camera_index >= len(cameras):
            raise RuntimeError(f"camera_index {camera_index} out of range (0..{len(cameras)-1})")
 
        self._cm = cm
        self._cam = cameras[camera_index]
        self._cam.acquire()
 
        cam_config = self._cam.generate_configuration([self._libcam.StreamRole.Viewfinder])
        stream_config = cam_config.at(0)
        stream_config.size = self._libcam.Size(capture_width, capture_height)
        try:
            stream_config.pixel_format = self._libcam.PixelFormat(prefer_pixel_format)
        except Exception:
            pass
 
        cam_config.validate()
        self._cam.configure(cam_config)
 
        self._stream_config = stream_config
        self._stream = stream_config.stream
 
        allocator = self._libcam.FrameBufferAllocator(self._cam)
        ret = allocator.allocate(self._stream)
        if ret <= 0:
            raise RuntimeError("Failed to allocate libcamera buffers")
 
        self._allocator = allocator
        self._reqs = []
        self._mapped = {}
 
        for buffer in allocator.buffers(self._stream):
            req = self._cam.create_request(0)
            req.add_buffer(self._stream, buffer)
            self._reqs.append(req)
            self._mapped[buffer] = self._MappedFrameBuffer(buffer).mmap()
 
        self._started = False
 
    def _import_mfb(self):
        from libcamera.utils import MappedFrameBuffer  # type: ignore
 
        return MappedFrameBuffer
 
    @property
    def event_fd(self) -> int:
        return int(self._cm.event_fd)
 
    @property
    def stream_info(self):
        return self._stream_config.size, self._stream_config.stride, str(self._stream_config.pixel_format)
 
    def start(self) -> None:
        if self._started:
            return
        self._cam.start()
        for req in self._reqs:
            self._cam.queue_request(req)
        self._started = True
 
    def stop(self) -> None:
        if not self._started:
            return
        try:
            self._cam.stop()
        finally:
            self._started = False
 
    def close(self) -> None:
        try:
            self.stop()
        finally:
            for mfb in self._mapped.values():
                try:
                    mfb.munmap()
                except Exception:
                    pass
            self._mapped.clear()
            try:
                self._cam.release()
            except Exception:
                pass
 
    def get_ready_requests(self):
        return self._cm.get_ready_requests()
 
    def requeue(self, req) -> None:
        req.reuse()
        self._cam.queue_request(req)
 
    def extract_rgb(self, req):
        buffers = req.buffers
        stream, fb = next(iter(buffers.items()))
        mfb = self._mapped[fb]
        plane = mfb.planes[0]
        size = self._stream_config.size
        width = int(size.width)
        height = int(size.height)
        stride = int(self._stream_config.stride)
        pf = str(self._stream_config.pixel_format)
 
        if pf == "RGB888":
            row_bytes = width * 3
        elif pf == "XRGB8888":
            row_bytes = width * 4
        else:
            raise RuntimeError(f"Unsupported pixel format: {pf}")
 
        if stride <= 0:
            stride = row_bytes
 
        buf = memoryview(plane)[: stride * height]
 
        try:
            import numpy as np  # type: ignore
        except Exception as e:
            raise RuntimeError(f"numpy not available: {e}")
 
        arr = np.frombuffer(buf, dtype=np.uint8).reshape((height, stride))
        arr = arr[:, :row_bytes]
        if pf == "RGB888":
            rgb = arr.reshape((height, width, 3))
            return rgb
        xrgb = arr.reshape((height, width, 4))
        rgb = xrgb[:, :, 1:4]
        return rgb
 
 
class LibcameraCameraNode(Node):
    def __init__(self) -> None:
        super().__init__("freenove_libcamera_camera")
 
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("topic", "/image_raw/compressed")
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("capture_width", 640)
        self.declare_parameter("capture_height", 480)
        self.declare_parameter("prefer_pixel_format", "RGB888")
        self.declare_parameter("stream_width", 400)
        self.declare_parameter("stream_height", 300)
        self.declare_parameter("hflip", False)
        self.declare_parameter("vflip", False)
        self.declare_parameter("publish_min_interval_s", 0.0)
        self.declare_parameter("jpeg_quality", 80)
 
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._topic = str(self.get_parameter("topic").value)
        self._camera_index = int(self.get_parameter("camera_index").value)
        self._capture_width = int(self.get_parameter("capture_width").value)
        self._capture_height = int(self.get_parameter("capture_height").value)
        self._prefer_pixel_format = str(self.get_parameter("prefer_pixel_format").value)
        self._stream_width = int(self.get_parameter("stream_width").value)
        self._stream_height = int(self.get_parameter("stream_height").value)
        self._hflip = bool(self.get_parameter("hflip").value)
        self._vflip = bool(self.get_parameter("vflip").value)
        self._publish_min_interval_s = float(self.get_parameter("publish_min_interval_s").value)
        self._jpeg_quality = int(self.get_parameter("jpeg_quality").value)
 
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(CompressedImage, self._topic, qos_profile)
 
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
 
    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        return super().destroy_node()
 
    def _encode_jpeg(self, rgb) -> Optional[bytes]:
        try:
            import cv2  # type: ignore
            import numpy as np  # type: ignore
 
            img = rgb
            if self._hflip:
                img = cv2.flip(img, 1)
            if self._vflip:
                img = cv2.flip(img, 0)
 
            if (self._stream_width > 0) and (self._stream_height > 0):
                img = cv2.resize(img, (self._stream_width, self._stream_height))
 
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            ok, enc = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(self._jpeg_quality)])
            if not ok:
                return None
            return enc.tobytes()
        except Exception:
            pass
 
        try:
            from PIL import Image as PILImage  # type: ignore
            import io
            import numpy as np  # type: ignore
 
            img = rgb
            if self._hflip:
                img = np.flip(img, axis=1)
            if self._vflip:
                img = np.flip(img, axis=0)
            if (self._stream_width > 0) and (self._stream_height > 0):
                pil = PILImage.fromarray(img, mode="RGB").resize((self._stream_width, self._stream_height))
            else:
                pil = PILImage.fromarray(img, mode="RGB")
            buf = io.BytesIO()
            pil.save(buf, format="JPEG", quality=int(self._jpeg_quality))
            return buf.getvalue()
        except Exception:
            return None
 
    def _run(self) -> None:
        capture = None
        selector = None
        last_pub = 0.0
        try:
            capture = _LibcameraCapture(
                capture_width=self._capture_width,
                capture_height=self._capture_height,
                camera_index=self._camera_index,
                prefer_pixel_format=self._prefer_pixel_format,
            )
            capture.start()
 
            selector = selectors.DefaultSelector()
            selector.register(capture.event_fd, selectors.EVENT_READ)
 
            while rclpy.ok() and (not self._stop_event.is_set()):
                events = selector.select(timeout=0.2)
                if not events:
                    continue
                reqs = capture.get_ready_requests()
                for req in reqs:
                    try:
                        rgb = capture.extract_rgb(req)
                        jpeg = self._encode_jpeg(rgb)
                        if jpeg is None:
                            continue
 
                        now = time.time()
                        if self._publish_min_interval_s > 0.0 and (now - last_pub) < self._publish_min_interval_s:
                            continue
                        last_pub = now
 
                        msg = CompressedImage()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = self._frame_id
                        msg.format = "jpeg"
                        msg.data = jpeg
                        self._pub.publish(msg)
                    finally:
                        capture.requeue(req)
        except Exception:
            self.get_logger().error(traceback.format_exc())
        finally:
            try:
                if selector is not None and capture is not None:
                    selector.unregister(capture.event_fd)
            except Exception:
                pass
            if capture is not None:
                try:
                    capture.close()
                except Exception:
                    pass
 
 
def main() -> None:
    rclpy.init()
    node = LibcameraCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
