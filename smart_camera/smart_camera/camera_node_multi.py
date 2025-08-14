# camera_node_multi.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class CameraNode(Node):
    def __init__(self, name, topic, video_path=None, image_path=None):
        super().__init__(name)
        self.publisher_ = self.create_publisher(Image, topic, 10)
        self.br = CvBridge()
        self.source_type = 'webcam'
        self.cap = None

        if video_path and os.path.exists(video_path):
            self.cap = cv2.VideoCapture(video_path)
            self.source_type = 'video'
            self.get_logger().info(f"[{name}] 📹 Video: {video_path}")
        elif image_path and os.path.exists(image_path):
            self.image = cv2.imread(image_path)
            self.source_type = 'image'
            self.get_logger().info(f"[{name}] 🖼 Image: {image_path}")
        else:
            self.cap = cv2.VideoCapture(0)
            self.get_logger().info(f"[{name}] 🎥 Webcam")

        self.timer = self.create_timer(0.03, self.timer_callback)

    def timer_callback(self):
        if self.source_type in ['video', 'webcam']:
            ret, frame = self.cap.read()
            if not ret:
                if self.source_type == 'video':
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ret, frame = self.cap.read()
                    if not ret:
                        return
                else:
                    return
        else:
            frame = self.image

        msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        super().destroy_node()
