# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage
# from std_msgs.msg import Bool  # Bool 타입으로 변경
# from std_msgs.msg import Float32MultiArray
# from cv_bridge import CvBridge
# import cv2
# from ultralytics import YOLO
# import numpy as np

# class ObjectDetectionSubscriber(Node):
#     def __init__(self):
#         super().__init__('object_detection_subscriber')
#         # CompressedImage 메시지 타입으로 구독
#         self.subscription = self.create_subscription(
#             CompressedImage,
#             'webcam/image_raw/compressed',  # 압축 이미지 토픽
#             self.listener_callback,
#             10
#         )

#         # 'object_detected'라는 이름의 토픽으로 Bool 타입 메시지 발행
#         self.object_detected_publisher = self.create_publisher(
#             Bool,  # Bool 타입으로 변경
#             'object_detected',
#             10
#         )
        
        
#         self.bounding_box_publisher = self.create_publisher(
#             Float32MultiArray,
#             'bounding_box/center',
#             10
#         )
        

#         self.bridge = CvBridge()
#         self.model = YOLO('car_best.pt')
#         self.get_logger().info('YOLOv8 모델이 로드되었습니다.')

#     def listener_callback(self, msg):
#         # 압축 이미지를 디코딩
#         np_arr = np.frombuffer(msg.data, np.uint8)  # 압축된 이미지 데이터를 numpy 배열로 변환
#         frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # JPEG 압축 해제

#         # YOLO 객체 탐지
#         results = self.model(frame)
#         annotated_frame = results[0].plot()

#         # 클래스 0이 감지되었는지 확인
#         class_0_detected = False

#         # 바운딩 박스 중심 좌표 계산 및 퍼블리시
#         centers = Float32MultiArray()
#         for box in results[0].boxes.data:
#             x1, y1, x2, y2, conf, cls = box.tolist()  # 바운딩 박스 좌표, 신뢰도, 클래스

#             # 클래스 0 감지 시 신호 발행
#             if cls == 0:
#                 class_0_detected = True

#             if cls == 0:
#                 name = 'car'
#             else:
#                 name = 'dummy'

#             # 바운딩 박스 그리기
#             cx = (x1 + x2) / 2  # 중심 x 좌표
#             cy = (y1 + y2) / 2  # 중심 y 좌표
#             height = (y2-y1)

#             centers.data.extend([cx, height])
#             self.get_logger().info(f'{name} , x좌표: ({cx}, 높이: {height})')

#             # 중심 좌표에 빨간 점 그리기
#             center = (int(cx), int(cy))
#             cv2.circle(annotated_frame, center, 5, (255, 0, 0), -1)  # 빨간색 점 (BGR)

#         # 클래스 0 객체가 감지되었을 때, Bool 메시지 발행
#         bool_msg = Bool()
#         bool_msg.data = class_0_detected  # 클래스 0이 감지되면 True, 아니면 False

#         # 'object_detected'라는 이름의 토픽에 메시지 발행
#         self.object_detected_publisher.publish(bool_msg)

#         # 중심 좌표 퍼블리시
#         if centers.data:
#             self.bounding_box_publisher.publish(centers)

#         line_color = (0, 255, 0)  # 초록색 (BGR)
#         line_thickness = 2  # 선 두께
#         height, width, _ = frame.shape

#         # x=280 선
#         cv2.line(annotated_frame, (240, 0), (240, height), line_color, line_thickness)
#         cv2.line(annotated_frame, (400, 0), (400, height), line_color, line_thickness)

#         # 결과 이미지 출력
#         cv2.imshow("YOLOv8 Object Detection", annotated_frame)
#         if cv2.waitKey(1) & 0xFF == ord('q'):
#             self.get_logger().info("종료 키 입력. 프로그램 종료.")
#             rclpy.shutdown()

# def main(args=None):
#     rclpy.init(args=args)
#     node = ObjectDetectionSubscriber()
#     rclpy.spin(node)
#     node.destroy_node()
#     cv2.destroyAllWindows()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool  # Bool 타입으로 변경
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np

class ObjectDetectionSubscriber(Node):
    def __init__(self):
        super().__init__('object_detection_subscriber')
        # CompressedImage 메시지 타입으로 구독
        self.subscription = self.create_subscription(
            CompressedImage,
            'webcam/image_raw/compressed',  # 압축 이미지 토픽
            self.listener_callback,
            10
        )

        # 'object_detected'라는 이름의 토픽으로 Bool 타입 메시지 발행
        self.object_detected_publisher = self.create_publisher(
            Bool,  # Bool 타입으로 변경
            'object_detected',
            10
        )
        
        # 'bounding_box/center'라는 이름의 토픽으로 중심 좌표 발행
        self.bounding_box_publisher = self.create_publisher(
            Float32MultiArray,
            'bounding_box/center',
            10
        )

        self.bridge = CvBridge()
        self.model = YOLO('car_best.pt')
        self.get_logger().info('YOLOv8 모델이 로드되었습니다.')

    def listener_callback(self, msg):
        # 압축 이미지를 디코딩
        np_arr = np.frombuffer(msg.data, np.uint8)  # 압축된 이미지 데이터를 numpy 배열로 변환
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # JPEG 압축 해제

        # YOLO 객체 탐지
        results = self.model(frame)
        annotated_frame = results[0].plot()

        # 클래스 0이 감지되었는지 확인
        class_0_detected = False

        # 바운딩 박스 중심 좌표 계산 및 퍼블리시
        centers = Float32MultiArray()
        for box in results[0].boxes.data:
            x1, y1, x2, y2, conf, cls = box.tolist()  # 바운딩 박스 좌표, 신뢰도, 클래스

            # 클래스 0 감지 시 신호 발행
            if cls == 0:
                class_0_detected = True

            if cls == 0:
                name = 'car'
            else:
                name = 'dummy'

            # 바운딩 박스 중심 좌표와 높이 계산
            cx = (x1 + x2) / 2  # 중심 x 좌표
            cy = (y1 + y2) / 2  # 중심 y 좌표
            height = (y2 - y1)

            centers.data.extend([cx, height])
            self.get_logger().info(f'{name}, x좌표: ({cx}), 높이: ({height})')

            # 중심 좌표에 빨간 점 그리기
            center = (int(cx), int(cy))
            cv2.circle(annotated_frame, center, 5, (255, 0, 0), -1)  # 빨간색 점 (BGR)

        # 객체가 감지되지 않았을 때 x값과 높이값을 0으로 고정
        if not centers.data:
            centers.data.extend([0.0, 0.0])
            self.get_logger().info('객체가 감지되지 않았습니다. x좌표: 0, 높이: 0')

        # 클래스 0 객체가 감지되었을 때, Bool 메시지 발행
        bool_msg = Bool()
        bool_msg.data = class_0_detected  # 클래스 0이 감지되면 True, 아니면 False
        self.object_detected_publisher.publish(bool_msg)

        # 중심 좌표 퍼블리시
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

