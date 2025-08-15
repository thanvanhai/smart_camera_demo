# smart_camera/smart_camera/person_verifier_node.py
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

from .face_utils import FaceEmbedder, load_enrolled_embeddings, verify

class PersonVerifierNode(Node):
    def __init__(self):
        super().__init__("person_verifier_node")
        self.bridge = CvBridge()

        # Params
        self.declare_parameter("camera_topics", ["/camera1/image_raw", "/camera2/image_raw"])
        self.declare_parameter("target_name", "Alice")  # người cần xác thực
        self.declare_parameter("embeddings_path", "data/embeddings.json")
        self.declare_parameter("threshold", 0.38)  # ArcFace cosine ~ 0.3–0.45 (tùy điều kiện)
        self.declare_parameter("providers", ["CPUExecutionProvider"])

        self.topics = self.get_parameter("camera_topics").get_parameter_value().string_array_value
        self.target_name = self.get_parameter("target_name").get_parameter_value().string_value
        self.emb_path = self.get_parameter("embeddings_path").get_parameter_value().string_value
        self.threshold = self.get_parameter("threshold").get_parameter_value().double_value
        providers = list(self.get_parameter("providers").get_parameter_value().string_array_value)

        self.face_embedder = FaceEmbedder(providers=providers)
        self.db = load_enrolled_embeddings(self.emb_path)
        if self.target_name not in self.db or len(self.db[self.target_name]) == 0:
            self.get_logger().warn(f"No embeddings for {self.target_name}. Please run enroll script.")

        self.pub = self.create_publisher(String, "/person_events", 10)
        self.subs = []
        for t in self.topics:
            self.subs.append(self.create_subscription(Image, t, self._cb(t), 10))
            self.get_logger().info(f"Subscribed to {t}")

    def _cb(self, topic_name):
        def inner(msg: Image):
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as e:
                self.get_logger().error(f"cv_bridge error: {e}")
                return

            faces = self.face_embedder.get_face_embeddings(frame)
            best = None
            best_score = -1.0

            enrolled = self.db.get(self.target_name, [])
            for (x1,y1,x2,y2), emb in faces:
                score = verify(emb, enrolled, agg="max")
                if score > best_score:
                    best_score = score
                    best = (x1,y1,x2,y2)

            if best is not None:
                (x1,y1,x2,y2) = best
                # Vẽ để debug (tùy bạn bật/tắt)
                vis = frame.copy()
                cv2.rectangle(vis, (x1,y1), (x2,y2), (0,255,0), 2)
                cv2.putText(vis, f"{self.target_name}: {best_score:.2f}", (x1, max(0,y1-10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                # Nếu muốn bật GUI:
                # cv2.imshow(f"Verify {topic_name}", vis)
                # cv2.waitKey(1)

                # Publish event nếu vượt ngưỡng
                verdict = (best_score >= self.threshold)
                event = {
                    "camera_topic": topic_name,
                    "target": self.target_name,
                    "score": round(best_score, 4),
                    "passed": verdict,
                    "bbox": [x1,y1,x2,y2]
                }
                self.pub.publish(String(data=json.dumps(event)))
        return inner

def main(args=None):
    rclpy.init(args=args)
    node = PersonVerifierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
