import rclpy
from rclpy.executors import MultiThreadedExecutor
from smart_camera.camera_node_rtsp import CameraNode
import sqlite3

DB_PATH = "camera.db"

def get_cameras_from_db():
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute("SELECT name, topic, video_path FROM cameras")
    rows = cursor.fetchall()
    conn.close()
    return rows

def main(args=None):
    rclpy.init(args=args)
    nodes = []

    cameras = get_cameras_from_db()
    for cam in cameras:
        name, topic, video_path = cam
        node = CameraNode(name, topic, video_path)
        nodes.append(node)

    # Dùng MultiThreadedExecutor thay cho spin_multi_threaded
    executor = MultiThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Shutting down camera nodes...")
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
