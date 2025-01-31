import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class ObjectDetectionSubscriber(Node):
    def __init__(self):
        super().__init__('object_detection_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'webcam/image_raw',
            self.listener_callback,
            10
        )
        self.bounding_box_publisher = self.create_publisher(
            Float32MultiArray,
            'bounding_box/center',
            10
        )
        self.bridge = CvBridge()
        self.model = YOLO('car_best.pt')
        self.get_logger().info('YOLOv8 모델이 로드되었습니다.')

    def listener_callback(self, msg):
        # 카메라 이미지 디코딩
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLO 객체 탐지
        results = self.model(frame)
        annotated_frame = results[0].plot()

        # 바운딩 박스 중심 좌표 계산 및 퍼블리시
        centers = Float32MultiArray()
        for box in results[0].boxes.data:
            x1, y1, x2, y2, conf, cls = box.tolist()  # 바운딩 박스 좌표, 신뢰도, 클래스
            cx = (x1 + x2) / 2  # 중심 x 좌표
            cy = (y1 + y2) / 2  # 중심 y 좌표
            centers.data.extend([cx, cy])
            self.get_logger().info(f'객체 중심: ({cx}, {cy})')

            # 중심 좌표에 빨간 점 그리기
            center = (int(cx), int(cy))
            cv2.circle(annotated_frame, center, 5, (255, 0, 0), -1)  # 빨간색 점 (BGR)

        # 중심 좌표 퍼블리시
        if centers.data:
            self.bounding_box_publisher.publish(centers)

        line_color = (0, 255, 0)  # 초록색 (BGR)
        line_thickness = 2  # 선 두께
        height, width, _ = frame.shape

        # x=280 선
        cv2.line(annotated_frame, (240, 0), (240, height), line_color, line_thickness)
        cv2.line(annotated_frame, (400, 0), (400, height), line_color, line_thickness)

        # 결과 이미지 출력
        cv2.imshow("YOLOv8 Object Detection", annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("종료 키 입력. 프로그램 종료.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
