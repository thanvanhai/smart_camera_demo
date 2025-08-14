import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import time

class CameraNode(Node):
    def __init__(self, name, topic, video_path, reconnect_interval=5.0):
        super().__init__(name)
        self.name = name
        self.topic = topic
        self.video_path = video_path
        self.reconnect_interval = reconnect_interval

        self.publisher = self.create_publisher(Image, self.topic, 10)
        self.bridge = CvBridge()
        self._stop_event = threading.Event()

        self.thread = threading.Thread(target=self._run)
        self.thread.start()
        self.get_logger().info(f"[{self.name}] Camera node started: {self.video_path}")

    def _run(self):
        while not self._stop_event.is_set():
            cap = cv2.VideoCapture(self.video_path, cv2.CAP_FFMPEG)
            if not cap.isOpened():
                self.get_logger().warning(f"[{self.name}] Cannot open stream, retry in {self.reconnect_interval}s")
                time.sleep(self.reconnect_interval)
                continue

            self.get_logger().info(f"[{self.name}] Stream opened successfully")
            while not self._stop_event.is_set():
                ret, frame = cap.read()
                if not ret:
                    self.get_logger().warning(f"[{self.name}] Lost connection, reconnecting...")
                    break

                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.publisher.publish(msg)
            cap.release()
            time.sleep(self.reconnect_interval)

    def destroy_node(self):
        self._stop_event.set()
        self.thread.join()
        super().destroy_node()
