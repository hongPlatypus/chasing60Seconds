import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
import time

class FollowingSystem(Node):
    def __init__(self):
        super().__init__('following_system')
        # Subscriber: 중심값 수신
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'bounding_box/center',
            self.listener_callback,
            10
        )
        # Publisher: 속도 명령
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        # 속성 초기화
        self.last_received_time = time.time()
        self.timeout_duration = 2.0  # 객체 사라짐 감지 시간 (초)
        self.image_width = 640    # 웹캠 해상도 가로 크기
        self.threshold = 50       # Threshold for distance from center
        self.angular_speed = 0.1  # 회전 속도
        self.linear_speed = 0.07  # 직진 속도
        self.log_interval = 1.0   # 로그 출력 간격 (초)
        self.last_log_time = time.time()  # 마지막 로그 출력 시간

    def listener_callback(self, msg):
        """수신된 메시지를 처리하고 로봇 동작을 제어합니다."""
        try:
            current_time = time.time()
            self.last_received_time = current_time
            data = msg.data
            # 데이터 유효성 검사
            if len(data) < 2:
                self.get_logger().warn("Received data is invalid or incomplete!")
                return  # 데이터가 잘못되었으면 함수 종료
            
            # 중심 좌표와 높이 값 추출
            center_x = data[0]
            height = data[1]
            
            # 로그 출력 (로그 간격을 유지)
            if current_time - self.last_log_time >= self.log_interval:
                self.get_logger().info(f'X좌표: {center_x}, 높이: {height}')
                self.last_log_time = current_time
            
            # 로봇 동작 제어
            twist = Twist()
            
            # x좌표가 0일 경우 로봇 멈춤
            if center_x == 0.0:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("객체가 감지되지 않아 로봇이 멈췄습니다.")
            else:
                # 각속도 제어
                if center_x < 240:
                    twist.angular.z = self.angular_speed  # 왼쪽으로 회전
                elif center_x > 400:
                    twist.angular.z = -self.angular_speed  # 오른쪽으로 회전
                else:
                    twist.angular.z = 0.0  # 정지
                
                # 선속도 제어
                if height < 165:
                    twist.linear.x = self.linear_speed  # 전진
                elif height > 175:
                    twist.linear.x = -self.linear_speed  # 후진
                else:
                    twist.linear.x = 0.0  # 정지
            
            # 최종 속도 명령 발행
            self.cmd_vel_publisher.publish(twist)
            
        except Exception as e:
            self.get_logger().error(f"Error in listener_callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = FollowingSystem()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down FollowingSystem node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
