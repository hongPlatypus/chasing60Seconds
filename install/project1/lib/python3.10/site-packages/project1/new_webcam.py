import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        # 'compressed' 토픽으로 CompressedImage 타입 메시지 발행
        self.publisher_ = self.create_publisher(CompressedImage, 'webcam/image_raw/compressed', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1초 주기로 실행
        self.cap = cv2.VideoCapture(0)  # 카메라 디바이스 ID
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # 이미지를 JPEG로 압축하여 메시지 생성
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = self.get_clock().now().to_msg()
            compressed_msg.format = "jpeg"
            compressed_msg.data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tobytes()  # JPEG 압축

            # 압축 이미지를 퍼블리시
            self.publisher_.publish(compressed_msg)
        else:
            self.get_logger().error('Failed to capture image from webcam.')

    def destroy_node(self):
        super().destroy_node()
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
