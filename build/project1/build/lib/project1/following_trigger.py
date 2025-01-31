import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import time

class BoundingBoxSubscriber(Node):
    def __init__(self):
        super().__init__('bounding_box_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'bounding_box/center',
            self.listener_callback,
            10
        )
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        self.last_log_time = time.time()  # 마지막으로 로그를 출력한 시간
        self.log_interval = 1.0  # 로그 출력 간격 (초)

    def listener_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_log_time >= self.log_interval:
            data = msg.data
            centers = [(int(data[i]), int(data[i + 1])) for i in range(0, len(data), 2)]
            self.get_logger().info(f'중심 좌표: {centers}')
            self.last_log_time = current_time  # 마지막 로그 출력 시간 업데이트

            # 중심 좌표에 따라 터틀봇의 이동 제어
            self.control_robot(centers)

    def control_robot(self, centers):
        # Twist 메시지 생성
        twist = Twist()

        # 중심 좌표가 여러 개일 수 있으므로 첫 번째 객체 중심만 사용
        if centers:
            cx, cy = centers[0]

            # 중심 좌표에 따른 조향 제어
            if cx < 240:
                twist.linear.x = 0.1  # 직진 속도
                twist.angular.z = 0.5  # 왼쪽 회전
            elif cx > 400:
                twist.linear.x = 0.1  # 직진 속도
                twist.angular.z = -0.5  # 오른쪽 회전
            else:
                twist.linear.x = 0.2  # 직진 속도
                twist.angular.z = 0.0  # 회전 없음 (중앙 맞추기)

            # 조향 명령 퍼블리시
            self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = BoundingBoxSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
