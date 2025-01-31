# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
# from geometry_msgs.msg import Twist
# import time

# class FollowingSystem(Node):
#     def __init__(self):
#         super().__init__('following_system')
#         # Subscriber: 중심값 수신
#         self.subscription = self.create_subscription(
#             Float32MultiArray,
#             'bounding_box/center',
#             self.listener_callback,
#             10
#         )
#         # Publisher: 속도 명령
#         self.cmd_vel_publisher = self.create_publisher(
#             Twist,
#             'cmd_vel',
#             10
#         )
#         # 속성 초기화
#         self.last_received_time = time.time()
#         self.timeout_duration = 2.0  # 객체 사라짐 감지 시간 (초)
#         self.image_width = 640  # 웹캠 해상도 가로 크기
#         self.threshold = 50  # Threshold for distance from center
#         self.angular_speed = 0.1  # 회전 속도
#         self.linear_speed = 0.05  # 직진 속도
#         self.log_interval = 1.0  # 로그 출력 간격 (초)
#         self.last_log_time = time.time()  # 마지막 로그 출력 시간

#     def listener_callback(self, msg):
#         """수신된 메시지를 처리하고 로봇 동작을 제어합니다."""
#         try:
#             current_time = time.time()
#             self.last_received_time = current_time
#             data = msg.data
#             # 데이터 유효성 검사
#             if len(data) < 2:
#                 self.get_logger().warn("Received data is invalid or incomplete!")
#                 return  # 데이터가 잘못되었으면 함수 종료
            
#             # 중심 좌표와 높이 값 추출
#             center_x = data[0]
#             height = data[1]
            
#             # 로그 출력 (로그 간격을 유지)
#             if current_time - self.last_log_time >= self.log_interval:
#                 self.get_logger().info(f'X좌표: {center_x}, 높이: {height}')
#                 self.last_log_time = current_time
            
#             # 로봇 동작 제어
#             twist = Twist()
            
#             # 각속도 제어
#             if center_x < 240:
#                 twist.angular.z = self.angular_speed  # 왼쪽으로 회전
#             elif center_x > 400:
#                 twist.angular.z = -self.angular_speed  # 오른쪽으로 회전
#             else:
#                 twist.angular.z = 0.0  # 정지
            
#             # 선속도 제어
#             if height < 145:
#                 twist.linear.x = self.linear_speed  # 전진
#             elif height > 155:
#                 twist.linear.x = -self.linear_speed  # 후진
#             else:
#                 twist.linear.x = 0.0  # 정지
            
#             # 최종 속도 명령 발행
#             self.cmd_vel_publisher.publish(twist)
            
#         except Exception as e:
#             self.get_logger().error(f"Error in listener_callback: {e}")

#     # def check_object_presence(self):
#     #     """객체 사라짐 감지."""
#     #     current_time = time.time()
#     #     if current_time - self.last_received_time > self.timeout_duration:
#     #         self.get_logger().warn("No object detected! Rotating in place.")
#     #         twist = Twist()
#     #         twist.angular.z = self.angular_speed  # 제자리에서 회전
#     #         self.cmd_vel_publisher.publish(twist)

# def main(args=None):
#     rclpy.init(args=args)
#     node = FollowingSystem()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Shutting down FollowingSystem node.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



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
                if height < 145:
                    twist.linear.x = self.linear_speed  # 전진
                elif height > 155:
                    twist.linear.x = -self.linear_speed  # 후진
                else:
                    twist.linear.x = 0.0  # 정지
            
            # 최종 속도 명령 발행
            self.cmd_vel_publisher.publish(twist)
            
        except Exception as e:
            self.get_logger().error(f"Error in listener_callback: {e}")

    # def check_object_presence(self):
    #     """객체 사라짐 감지."""
    #     current_time = time.time()
    #     if current_time - self.last_received_time > self.timeout_duration:
    #         self.get_logger().warn("No object detected! Rotating in place.")
    #         twist = Twist()
    #         twist.angular.z = self.angular_speed  # 제자리에서 회전
    #         self.cmd_vel_publisher.publish(twist)

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



# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
# from geometry_msgs.msg import Twist
# import time

# class VelocityController:
#     def __init__(self, h_target, v_base, k_p, v_min, v_max):
#         self.h_target = h_target  # 목표 높이값
#         self.v_base = v_base      # 기본 속도
#         self.k_p = k_p            # 비례 계수
#         self.v_min = v_min        # 최소 속도
#         self.v_max = v_max        # 최대 속도

#     def compute_velocity(self, h_current):
#         # 오차 계산
#         error = self.h_target - h_current
#         # 보정값 계산
#         v_adjust = self.k_p * error
#         # 최종 속도 계산
#         v_final = self.v_base + v_adjust
#         # 속도 제한 적용
#         v_final = max(self.v_min, min(v_final, self.v_max))
#         return v_final

# class FollowingSystem(Node):
#     def __init__(self):
#         super().__init__('following_system')
#         # Subscriber: 중심값 수신
#         self.subscription = self.create_subscription(
#             Float32MultiArray,
#             'bounding_box/center',
#             self.listener_callback,
#             10
#         )
#         # Publisher: 속도 명령
#         self.cmd_vel_publisher = self.create_publisher(
#             Twist,
#             'cmd_vel',
#             10
#         )
#         self.last_received_time = time.time()
#         self.timeout_duration = 2.0  # 객체 사라짐 감지 시간 (초)
#         self.image_width = 640  # 웹캠 해상도 가로 크기
#         self.threshold = 50  # Threshold for distance from center
#         self.angular_speed = 0.5  # 회전 속도
#         self.linear_speed = 0.2  # 직진 속도
#         self.timer = self.create_timer(0.1, self.check_object_presence)
#         self.log_interval = 1.0  # 로그 출력 간격 (초)
#         self.last_log_time = time.time()  # 마지막 로그 출력 시간

#         # Velocity Controller 초기화
#         self.velocity_controller = VelocityController(
#             h_target=1.5,  # 목표 높이값 (예: 1.5m)
#             v_base=0.5,    # 기본 속도 (예: 0.5 m/s)
#             k_p=0.1,       # 비례 계수 (예: 0.1)
#             v_min=0.1,     # 최소 속도
#             v_max=1.0      # 최대 속도
#         )

#     def listener_callback(self, msg):
#         current_time = time.time()
#         self.last_received_time = current_time
#         data = msg.data
#         # 데이터 유효성 검사
#         if not data or len(data) < 2:
#             self.get_logger().warn("수신된 데이터가 올바르지 않습니다.")
#             return

#         center_x = data[0] * self.image_width
#         current_height = data[1]  # 높이값 추출

#         # 로그 출력 (주기적)
#         if current_time - self.last_log_time >= self.log_interval:
#             self.get_logger().info(f'중심 좌표: {center_x}, 현재 높이: {current_height}')
#             self.last_log_time = current_time

#         # 중심값과 높이를 기반으로 로봇 동작 제어
#         self.control_robot(center_x, current_height)

#     def control_robot(self, center_x, current_height):
#         twist = Twist()

#         # 중심값과 화면 중앙의 거리 계산
#         error = (center_x - self.image_width / 2) / (self.image_width / 2)

#         # 동적 속도 조정 (높이값 보정 포함)
#         twist.linear.x = self.velocity_controller.compute_velocity(current_height)
#         if center_x < self.image_width * 0.4 - self.threshold:
#             twist.angular.z = self.angular_speed * error  # 왼쪽으로 조향하며 직진
#         elif center_x > self.image_width * 0.6 + self.threshold:
#             twist.angular.z = self.angular_speed * error  # 오른쪽으로 조향하며 직진
#         else:
#             twist.angular.z = 0.0  # 직진

#         self.cmd_vel_publisher.publish(twist)

#     def check_object_presence(self):
#         current_time = time.time()
#         if current_time - self.last_received_time > self.timeout_duration:
#             # 객체가 사라진 경우: 탐색 동작
#             twist = Twist()
#             twist.linear.x = 0.1  # 느리게 전진
#             twist.angular.z = self.angular_speed  # 왼쪽 회전
#             self.get_logger().warn("객체가 감지되지 않습니다. 탐색 중...")
#             self.cmd_vel_publisher.publish(twist)
#         else:
#             # 객체가 감지된 경우: FollowingSystem 로직 유지
#             self.get_logger().info("객체가 감지되었습니다. 로직을 유지합니다.")

#             # listener_callback이 정상적으로 호출될 수 있도록 트리거 역할 수행
#             last_data = Float32MultiArray()
#             last_data.data = [0.5, 1.5]  # 예제 데이터로 최신 값 시뮬레이션
#             self.listener_callback(last_data)

# def main(args=None):
#     rclpy.init(args=args)
#     node = FollowingSystem()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Shutting down FollowingSystem node.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
# from geometry_msgs.msg import Twist
# import time

# class FollowingSystem(Node):
#     def __init__(self):
#         super().__init__('following_system')
#         self.subscription = self.create_subscription(
#             Float32MultiArray,
#             'bounding_box/center',
#             self.listener_callback,
#             10
#         )
#         self.cmd_vel_publisher = self.create_publisher(
#             Twist,
#             'cmd_vel',
#             10
#         )
#         self.last_received_time = time.time()
#         self.timeout_duration = 0.5
#         self.image_width = 640
#         self.threshold = 50
#         self.angular_speed = 0.1
#         self.linear_speed = 0.05
#         self.log_interval = 1.0
#         self.last_log_time = time.time()
#         self.last_data = None

#     def listener_callback(self, msg):
#         try:
#             current_time = time.time()
#             self.last_received_time = current_time
#             data = msg.data
#             if not data or len(data) < 2:
#                 self.get_logger().warn("수신된 데이터가 올바르지 않습니다.")
#                 return
#             self.last_data = data
#             center_x = data[0]
#             height = data[1]
#             if current_time - self.last_log_time >= self.log_interval:
#                 self.get_logger().info(f'X좌표: {center_x}, 높이: {height}')
#                 self.last_log_time = current_time
#             twist = Twist()
#             if center_x < 240:
#                 twist.angular.z = self.angular_speed
#             elif center_x > 400:
#                 twist.angular.z = -self.angular_speed
#             else:
#                 twist.angular.z = 0.0
#             if height < 145:
#                 twist.linear.x = self.linear_speed
#             elif height > 155:
#                 twist.linear.x = -self.linear_speed
#             else:
#                 twist.linear.x = 0.0
#             self.cmd_vel_publisher.publish(twist)
#         except Exception as e:
#             self.get_logger().error(f"Error in listener_callback: {e}")

#     def check_object_presence(self):
#         current_time = time.time()
#         self.get_logger().error(f"확인용: {current_time - self.last_received_time}")
#         if current_time - self.last_received_time > self.timeout_duration:
#             twist = Twist()
#             twist.linear.x = 0.0
#             twist.angular.z = 0.0
#             self.get_logger().warn("객체가 감지되지 않습니다. 정지!!!")
#             self.cmd_vel_publisher.publish(twist)
#         else:
#             if self.last_data is not None:
#                 self.get_logger().info("객체가 감지되었습니다. 로직을 유지")
#                 last_data_msg = Float32MultiArray()
#                 last_data_msg.data = self.last_data
#                 self.listener_callback(last_data_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = FollowingSystem()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Shutting down FollowingSystem node.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
