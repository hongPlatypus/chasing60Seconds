import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from std_msgs.msg import Bool
import time


class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('initial_pose_publisher')
        # Publisher for /initialpose
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # Subscriber for /patrol_start
        self.patrol_start_subscriber = self.create_subscription(
            Bool,
            '/patrol_start',
            self.patrol_start_callback,
            10
        )
        # State variable to prevent multiple triggers
        self.patrol_start_received = False

    def patrol_start_callback(self, msg):

        """Callback to handle /patrol_start messages."""

        if msg.data and not self.patrol_start_received:
            self.patrol_start_received = True
            self.get_logger().info('/patrol_start received as True. Publishing initial pose in 5 seconds...')
            # Schedule the initial pose publication in 5 seconds
            time.sleep(5)
            self.publish_initial_pose()


    def publish_initial_pose(self):
        """Publishes the initial pose."""
        # Create the initial pose message
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()

        # Set position
        initial_pose.pose.pose.position.x = 0.1750425100326538
        initial_pose.pose.pose.position.y = 0.05808566138148308
        initial_pose.pose.pose.position.z = 0.0

        # Set orientation

        initial_pose.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=-0.04688065682721989,
            w=0.9989004975549108
        )
        # Set covariance
        initial_pose.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
        ]

        # Publish the message
        self.publisher.publish(initial_pose)
        self.get_logger().info('Initial pose published.')


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()