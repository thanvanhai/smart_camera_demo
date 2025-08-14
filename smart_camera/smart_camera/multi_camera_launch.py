def main(args=None):
    import rclpy
    from smart_camera.camera_node_multi import CameraNode
    from smart_camera.yolo_multi_subscriber import YoloMultiSubscriber

    rclpy.init(args=args)

    # Tạo các camera node
    cam1 = CameraNode('cam1', '/camera1/image_raw', video_path='video1.mp4')
    cam2 = CameraNode('cam2', '/camera2/image_raw', video_path='video2.mp4')

    # Tạo YOLO subscriber lắng nghe cả 2 topic
    yolo_node = YoloMultiSubscriber(['/camera1/image_raw', '/camera2/image_raw'])

    # Dùng MultiThreadedExecutor để chạy đồng thời
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(cam1)
    executor.add_node(cam2)
    executor.add_node(yolo_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        cam1.destroy_node()
        cam2.destroy_node()
        yolo_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
