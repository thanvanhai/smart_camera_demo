Hệ thống **Multi-Camera Smart Camera** sử dụng ROS2 + OpenCV + YOLOv8 để nhận diện đối tượng từ nhiều camera hoặc video.
## 📂 Cấu trúc thư mục
```
smart_camera_demo/
├─ smart_camera/                 # ROS2 package
│  ├─ smart_camera/              # Code Python
│  │  ├─ __init__.py
│  │  ├─ camera_node.py          # Node phát video/webcam
│  │  ├─ yolo_multi_subscriber.py# Node YOLO nhận dạng
│  │  └─ multi_camera_launch.py  # Launch nhiều camera
│  ├─ package.xml
│  └─ setup.py
├─ videos/                        # Thư mục chứa video test
└─ README.md


---

## ⚙️ Yêu cầu

- **WSL2 trên Windows 10/11** với WSLg (GUI hỗ trợ)
- **Ubuntu 22.04 LTS** (hoặc tương thích ROS2 Humble)
- **ROS2 Humble** đã cài đặt
- Python 3.10+
- OpenCV, Ultralytics YOLOv8

---

## 🛠 Cài đặt ROS2 và phụ thuộc

1. **Cài ROS2 Humble**: [Hướng dẫn chính thức](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)  
2. Cài các package hỗ trợ ROS2 Python:

```bash
sudo apt update
sudo apt install -y ros-humble-cv-bridge ros-humble-image-transport python3-colcon-common-extensions

Tạo virtualenv (tuỳ chọn) và cài YOLO + OpenCV:

python3 -m venv venv
source venv/bin/activate
pip install opencv-python ultralytics

Chạy hệ thống Multi-Camera

Build package ROS2:

cd ~/smart_camera_demo
colcon build --symlink-install
source install/setup.bash


Chạy node launch:

ros2 run smart_camera multi_camera_launch
Cấu hình camera/video

Trong multi_camera_launch.py:

cam1 = CameraNode('cam1', '/camera1/image_raw', video_path='videos/video1.mp4')
cam2 = CameraNode('cam2', '/camera2/image_raw', video_path='videos/video2.mp4')


video_path có thể là đường dẫn file .mp4.

Nếu muốn dùng webcam, thay video_path=''.

🖼 Giao diện YOLO

Node YOLO sẽ mở cửa sổ GUI WSLg hiển thị kết quả nhận dạng.

Cảnh báo "user config directory not writeable" là bình thường, có thể tạo thư mục:

mkdir -p ~/.config/Ultralytics
chmod 700 ~/.config/Ultralytics

