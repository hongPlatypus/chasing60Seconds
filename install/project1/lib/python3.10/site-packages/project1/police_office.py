# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Bool, String

# class PoliceOffice(Node):
#     def __init__(self):
#         super().__init__('police_office')
#         self.patrol_start_publisher = self.create_publisher(Bool, '/patrol_start', 10)
#         self.start_detection_publisher = self.create_publisher(Bool, '/starting_detection', 10)
#         self.object_detected_subscriber = self.create_subscription(
#             String,
#             '/object_detected',
#             self.object_detected_callback,
#             10
#         )
#         self.publish_messages()

#     def publish_messages(self):
#         start_msg = Bool()
#         start_msg.data = True
#         self.patrol_start_publisher.publish(start_msg)
#         self.get_logger().info("자동 발행: /patrol_start 신호 발행.")
#         self.start_detection_publisher.publish(start_msg)
#         self.get_logger().info("자동 발행: /starting_detection 신호 발행.")

#     def object_detected_callback(self, msg):
#         self.get_logger().info(f"탐지된 객체: {msg.data}")

# def main(args=None):
#     rclpy.init(args=args)
#     police_office_node = PoliceOffice()
#     rclpy.spin(police_office_node)
#     police_office_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class PoliceOffice(Node):
    def __init__(self):
        super().__init__('police_office')
        self.patrol_start_publisher = self.create_publisher(Bool, '/patrol_start', 10)
        self.publish_patrol_start_signal()

    def publish_patrol_start_signal(self):
        start_msg = Bool()
        start_msg.data = True
        self.patrol_start_publisher.publish(start_msg)
        self.get_logger().info("자동 발행: /patrol_start 신호 발행.")

def main(args=None):
    rclpy.init(args=args)
    police_office_node = PoliceOffice()
    rclpy.spin(police_office_node)
    police_office_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
