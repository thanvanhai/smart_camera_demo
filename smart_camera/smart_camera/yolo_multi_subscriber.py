import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import sqlite3

DB_PATH = "camera.db"

class YOLONode(Node):
    def __init__(self, camera_topic, output_topic=None):
        super().__init__(f"yolo_{camera_topic.replace('/', '_')}")
        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, camera_topic, self.callback, 10)
        self.output_topic = output_topic
        if output_topic:
            self.pub = self.create_publisher(Image, output_topic, 10)
        else:
            self.pub = None
        self.model = YOLO("yolov8n.pt")  # load YOLOv8 nano model

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(frame)[0]
        annotated_frame = results.plot()
        if self.pub:
            out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            self.pub.publish(out_msg)

def get_camera_topics():
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("SELECT topic FROM cameras")
    rows = cursor.fetchall()
    conn.close()
    return [r[0] for r in rows]

def main():
    rclpy.init()
    nodes = []
    for topic in get_camera_topics():
        node = YOLONode(topic, output_topic=topic.replace('image_raw', 'yolo'))
        nodes.append(node)

    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
