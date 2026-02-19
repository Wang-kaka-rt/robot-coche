import time
from threading import Condition

import rclpy
from sensor_msgs.msg import CompressedImage

class Camera:
    def __init__(
        self,
        node,
        input_topic: str = "/camera/image_raw/compressed",
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

    def _on_image(self, msg: CompressedImage) -> None:
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
            self._subscription = self._node.create_subscription(
                CompressedImage, self._input_topic, self._on_image, 10
            )
            self.streaming = True

    def stop_stream(self) -> None:
        if self.streaming:
            try:
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
