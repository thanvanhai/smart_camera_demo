import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_subscriber')
        self.subscriber = self.create_subscription(
            Image, 
            '/camera/image_raw',  # topic từ camera_node
            self.listener_callback, 
            10
        )
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # model nhỏ cho nhanh

    def listener_callback(self, data):
        # Chuyển ROS Image -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        
        # Chạy YOLOv8 inference
        results = self.model(frame)
        annotated = results[0].plot()  # vẽ bounding box

        # Hiển thị GUI
        cv2.imshow("YOLO Detection", annotated)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("🛑 GUI đóng theo yêu cầu")
            rclpy.shutdown()

    def destroy_node(self):
        # Giải phóng OpenCV window
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = YoloSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 KeyboardInterrupt - Thoát")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
