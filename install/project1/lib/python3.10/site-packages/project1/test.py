import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class ObjectDetectionSubscriber(Node):
    def __init__(self):
        super().__init__('object_detection_listener')
        
        # 'object_detected' 토픽을 구독하고, 콜백 함수로 listener_callback을 설정
        self.subscription = self.create_subscription(
            Bool,
            'object_detected',  # 객체 감지 신호 토픽
            self.listener_callback,
            10
        )
        self.get_logger().info('Object Detection Listener가 시작되었습니다.')

    def listener_callback(self, msg):
        # 수신한 메시지가 True일 때 "카트가 감지되었습니다." 출력
        if msg.data:
            self.get_logger().info('카트가 감지되었습니다.')
        else:
            self.get_logger().info('카트가 감지되지 않았습니다.')

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
