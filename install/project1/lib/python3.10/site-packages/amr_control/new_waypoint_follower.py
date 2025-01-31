import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints
from std_msgs.msg import String
import math
import sys

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.initial_pose_sub = self.create_subscription(String, '/initialpose', self.initial_pose_callback, 10)
        self.object_detected_sub = self.create_subscription(String, '/object_detected', self.object_detected_callback, 10)
        self.follow_publisher = self.create_publisher(String, '/follow', 10)
        self._goal_handle = None  # Ensure the goal handle is defined

        # Automatically send goal upon initialization
        self.get_logger().info('Sending goal...')
        self.send_goal()

    def initial_pose_callback(self, msg):
        self.get_logger().info(f'Received initial pose: {msg.data}')

    def object_detected_callback(self, msg):
        self.get_logger().info(f'Object detected: {msg.data}')

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Convert Euler angles to a quaternion
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def send_goal(self):
        # Example waypoints
        waypoints = []

        # Waypoint 1
        waypoint1 = PoseStamped()
        waypoint1.header.frame_id = "map"
        waypoint1.pose.position.x = 0.20662596156342655
        waypoint1.pose.position.y = -0.4871472785139167
        waypoint1.pose.position.z = 0.0
        # waypoint1.pose.orientation.x = 0.0
        # waypoint1.pose.orientation.y = 0.0
        # waypoint1.pose.orientation.z = -0.7925213193702164
        # waypoint1.pose.orientation.w = 0.6098442082562492

        waypoint1_yaw = -1.17  # Target orientation in radians
        waypoint1.pose.orientation = self.euler_to_quaternion(0, 0, waypoint1_yaw)
        waypoints.append(waypoint1)
        
        # Waypoint 2
        waypoint2 = PoseStamped()
        waypoint2.header.frame_id = "map"
        waypoint2.pose.position.x = -0.5098931522807715
        waypoint2.pose.position.y = -0.5766481689018857
        waypoint2.pose.position.z = 0.0
        # waypoint2.pose.orientation.x = 0.0
        # waypoint2.pose.orientation.y = 0.0
        # waypoint2.pose.orientation.z = 0.9732097821871202
        # waypoint2.pose.orientation.w = 0.22991894192366602
        waypoint2_yaw = 3.017  # Target orientation in radians
        waypoint2.pose.orientation = self.euler_to_quaternion(0, 0, waypoint2_yaw)
        waypoints.append(waypoint2)

        # Waypoint 3
        waypoint3 = PoseStamped()
        waypoint3.header.frame_id = "map"
        waypoint3.pose.position.x = -0.7339190729114683
        waypoint3.pose.position.y = -0.366880572197456
        waypoint3.pose.position.z = 0.0
        # waypoint3.pose.orientation.x = 0.0
        # waypoint3.pose.orientation.y = 0.0
        # waypoint3.pose.orientation.z = 0.9737618891377401
        # waypoint3.pose.orientation.w = 0.22756929332161577
        waypoint3_yaw = 2.384  # Target orientation in radians
        waypoint3.pose.orientation = self.euler_to_quaternion(0, 0, waypoint3_yaw)
        waypoints.append(waypoint3)

        # Waypoint 4
        waypoint4 = PoseStamped()
        waypoint4.header.frame_id = "map"
        waypoint4.pose.position.x = -1.2838784688896316
        waypoint4.pose.position.y = -0.2265589907963823
        waypoint4.pose.position.z = 0.0
        # waypoint4.pose.orientation.x = 0.0
        # waypoint4.pose.orientation.y = 0.0
        # waypoint4.pose.orientation.z = -0.8604881998633004
        # waypoint4.pose.orientation.w = 0.5094703699883014
        waypoint4_yaw = 2.893  # Target orientation in radians
        waypoint4.pose.orientation = self.euler_to_quaternion(0, 0, waypoint4_yaw)
        waypoints.append(waypoint4)

        # Waypoint 5
        waypoint5 = PoseStamped()
        waypoint5.header.frame_id = "map"
        waypoint5.pose.position.x = -1.4204211936158202
        waypoint5.pose.position.y = -0.5974287556500998
        waypoint5.pose.position.z = 0.0
        # waypoint5.pose.orientation.x = 0.0
        # waypoint5.pose.orientation.y = 0.0
        # waypoint5.pose.orientation.z = -0.7355195642029945
        # waypoint5.pose.orientation.w = 0.6775034838837636
        waypoint5_yaw = -1.930  # Target orientation in radians
        waypoint5.pose.orientation = self.euler_to_quaternion(0, 0, waypoint5_yaw)
        waypoints.append(waypoint5)


          # Waypoint6
        waypoint6 = PoseStamped()
        waypoint6.header.frame_id = "map"
        waypoint6.pose.position.x = -1.5243525436303194
        waypoint6.pose.position.y = -0.6399604662931665
        waypoint6.pose.position.z = 0.0
        # waypoint5.pose.orientation.x = 0.0
        # waypoint5.pose.orientation.y = 0.0
        # waypoint5.pose.orientation.z = -0.7355195642029945
        # waypoint5.pose.orientation.w = 0.6775034838837636
        waypoint6_yaw = -2.748  # Target orientation in radians
        waypoint6.pose.orientation = self.euler_to_quaternion(0, 0, waypoint6_yaw)
        waypoints.append(waypoint6)

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        self.action_client.wait_for_server()

        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current Waypoint Index: {feedback.current_waypoint}')

    def get_result_callback(self, future):
        result = future.result().result
        missed_waypoints = result.missed_waypoints
        if missed_waypoints:
            self.get_logger().info(f'Missed waypoints: {missed_waypoints}')
        else:
            self.get_logger().info('All waypoints completed successfully!')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
