import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.br = CvBridge()

        # Tham số ROS2
        self.declare_parameter('video_path', '')
        self.declare_parameter('image_path', '')

        video_path = self.get_parameter('video_path').get_parameter_value().string_value
        image_path = self.get_parameter('image_path').get_parameter_value().string_value

        # Chọn nguồn phát
        if video_path and os.path.exists(video_path):
            self.cap = cv2.VideoCapture(video_path)
            self.source_type = 'video'
            self.get_logger().info(f"📹 Phát video từ file: {video_path}")
        elif image_path and os.path.exists(image_path):
            self.image = cv2.imread(image_path)
            self.source_type = 'image'
            self.get_logger().info(f"🖼 Phát ảnh từ file: {image_path}")
        else:
            self.cap = cv2.VideoCapture(0)
            self.source_type = 'webcam'
            self.get_logger().info("🎥 Phát từ webcam")

        self.timer = self.create_timer(0.03, self.timer_callback)

    def timer_callback(self):
        if self.source_type in ['video', 'webcam']:
            ret, frame = self.cap.read()
            if not ret:
                if self.source_type == 'video':
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Lặp lại video
                    ret, frame = self.cap.read()
                    if not ret:
                        self.get_logger().error("❌ Không đọc được video")
                        return
                else:
                    self.get_logger().error("❌ Không đọc được webcam")
                    return
        else:  # image
            frame = self.image

        # Hiển thị GUI
        cv2.imshow("Camera Feed", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("🛑 Đóng GUI theo yêu cầu")
            rclpy.shutdown()
            return

        # Publish ROS Image
        msg = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

    def destroy_node(self):
        # Giải phóng camera và đóng cửa sổ GUI
        if self.source_type in ['video', 'webcam'] and self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 KeyboardInterrupt - Thoát")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
