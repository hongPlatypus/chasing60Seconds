import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import FollowWaypoints
import math
import threading
import sys
import select
import termios
import tty

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Convert Euler angles to a quaternion
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def send_goal(self):
        # 세 개의 웨이포인트 정의
        waypoints = []
     # 1. 직진 이동 (첫 번째 웨이포인트)
        waypoint1 = PoseStamped()
        waypoint1.header.frame_id = "map"
        waypoint1.pose.position.x = 0.31342234836722255
        waypoint1.pose.position.y = -0.08484887168959171
        waypoint1.pose.position.z = 0.0
        waypoint1.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        waypoints.append(waypoint1)

        # 2. 회전 (첫 번째 목표)
        waypoint2 = PoseStamped()
        waypoint2.header.frame_id = "map"
        waypoint2.pose.position.x = 0.2723321993437609
        waypoint2.pose.position.y = -0.04381486078206857
        waypoint2.pose.position.z = 0.0
        waypoint2.pose.orientation = Quaternion(x=0.0, y=0.0, z=-0.7117271332073097, w=0.7024560398035626)
        waypoints.append(waypoint2)

        # 3. 직진 이동 (두 번째 웨이포인트)
        waypoint3 = PoseStamped()
        waypoint3.header.frame_id = "map"
        waypoint3.pose.position.x = 0.27915079254778574
        waypoint3.pose.position.y =  -0.4819171959152196
        waypoint3.pose.position.z = 0.0
        waypoint3.pose.orientation = Quaternion(x=0.0, y=0.0, z=-0.7268616948123648, w=0.6867838645560164)
        waypoints.append(waypoint3)

        # 4. 회전 (두 번째 목표)
        waypoint4 = PoseStamped()
        waypoint4.header.frame_id = "map"
        waypoint4.pose.position.x = 0.2927166296238926
        waypoint4.pose.position.y = -0.4884538165939268
        waypoint4.pose.position.z = 0.0
        waypoint4.pose.orientation = Quaternion(x=0.0, y=0.0, z=-0.9989800697923973, w=0.045153296198361746)
        waypoints.append(waypoint4)

        # 5. 직진 (세 번째 목표)
        waypoint5 = PoseStamped()
        waypoint5.header.frame_id = "map"
        waypoint5.pose.position.x = -0.606589093051801
        waypoint5.pose.position.y = -0.6125996744477369
        waypoint5.pose.position.z = 0.0
        waypoint5.pose.orientation = Quaternion(x=0.0, y=0.0, z=-0.9989800697923973, w=0.045153296198361746)
        waypoints.append(waypoint5)

        # 6. 회전 (세 번째 목표)
        waypoint6 = PoseStamped()
        waypoint6.header.frame_id = "map"
        waypoint6.pose.position.x = -0.606589093051801
        waypoint6.pose.position.y = -0.6125996744477369
        waypoint6.pose.position.z = 0.0
        waypoint6.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.7789033198865722, w=0.7789033198865722)
        waypoints.append(waypoint6)

        # 7. 직진 (네 번째 목표)
        waypoint7 = PoseStamped()
        waypoint7.header.frame_id = "map"
        waypoint7.pose.position.x = -0.6228302994497886
        waypoint7.pose.position.y = -0.32845921701984865
        waypoint7.pose.position.z = 0.0
        waypoint7.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.7789033198865722, w=0.7789033198865722)
        waypoints.append(waypoint7)

        # 8. 회전 (네 번째 목표)
        waypoint8 = PoseStamped()
        waypoint8.header.frame_id = "map"
        waypoint8.pose.position.x = -0.6228302994497886
        waypoint8.pose.position.y = -0.32845921701984865
        waypoint8.pose.position.z = 0.0
        waypoint8.pose.orientation = Quaternion(x=0.0, y=0.0, z=-0.9927597744437975, w=0.12011673591261282)
        waypoints.append(waypoint8)


        # # 8. 회전 (네 번째 목표)
        # waypoint9 = PoseStamped()
        # waypoint9.header.frame_id = "map"
        # waypoint9.pose.position.x = -0.6228302994497886
        # waypoint9.pose.position.y = -0.39845921701984865
        # waypoint9.pose.position.z = 0.0
        # waypoint9.pose.orientation = Quaternion(x=0.0, y=0.0, z=-0.9995559408389308, w=0.02979800553056823)
        # waypoints.append(waypoint9)


        # # 8. 회전 (네 번째 목표)
        # waypoint10 = PoseStamped()
        # waypoint10.header.frame_id = "map"
        # waypoint10.pose.position.x = -0.6228302994497886
        # waypoint10.pose.position.y = -0.39845921701984865
        # waypoint10.pose.position.z = 0.0
        # waypoint10.pose.orientation = Quaternion(x=0.0, y=0.0, z=-0.9995559408389308, w=0.02979800553056823)
        # waypoints.append(waypoint10)


        # FollowWaypoints 액션 목표 생성 및 전송
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        # 서버 연결 대기
        self.action_client.wait_for_server()

        # 목표 전송 및 피드백 콜백 설정
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
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Current Waypoint Index: {feedback.current_waypoint}')

    def cancel_goal(self):
        if self._goal_handle is not None:
            self.get_logger().info('Attempting to cancel the goal...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().info('No active goal to cancel.')

    # def cancel_done_callback(self, future):
    #     cancel_response = future.result()
    #     if cancel_response.accepted:
    #         self.get_logger().info('Goal cancellation accepted. Exiting program...')
    #         self.destroy_node()
    #         rclpy.shutdown()
    #         sys.exit(0)  # Exit the program after successful cancellation
    #     else:
    #         self.get_logger().info('Goal cancellation failed or no active goal to cancel.')

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_cancelled) > 0:
            self.get_logger().info('Goal cancellation accepted. Exiting program...')
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(0)  # Exit the program after successful cancellation
        else:
            self.get_logger().info('Goal cancellation failed or no active goal to cancel.')

    def get_result_callback(self, future):
        result = future.result().result
        missed_waypoints = result.missed_waypoints
        if missed_waypoints:
            self.get_logger().info(f'Missed waypoints: {missed_waypoints}')
        else:
            self.get_logger().info('All waypoints completed successfully!')

def keyboard_listener(node):
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while True:
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key.lower() == 'g':
                    node.get_logger().info('Key "g" pressed. Sending goal...')
                    node.send_goal()
                elif key.lower() == 's':
                    node.get_logger().info('Key "s" pressed. Cancelling goal...')
                    node.cancel_goal()
                    break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    
    thread = threading.Thread(target=keyboard_listener, args=(node,), daemon=True)
    thread.start()
    
    rclpy.spin(node)


if __name__ == '__main__':
    main()