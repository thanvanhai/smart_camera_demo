# yolo_multi_subscriber.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloMultiSubscriber(Node):
    def __init__(self, topics):
        super().__init__('yolo_multi_subscriber')
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.subscribers = []

        for topic in topics:
            sub = self.create_subscription(
                Image,
                topic,
                lambda data, t=topic: self.callback(data, t),
                10
            )
            self.subscribers.append(sub)

    def callback(self, data, topic):
        frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        results = self.model(frame)
        annotated = results[0].plot()
        cv2.imshow(f"YOLO - {topic}", annotated)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()
