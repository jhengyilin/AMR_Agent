#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from threading import Condition

class Nav2Commander(Node):
    def __init__(self):
        super().__init__('nav2_commander')
        qos = QoSProfile(depth=10)

        # Publisher for /current_pose
        self.pose_publisher_ = self.create_publisher(PoseStamped, '/current_pose', qos)
        self.get_logger().info('Publisher created for /current_pose')

        # Subscription to /amcl_pose topic
        self.amcl_subscription = self.create_subscription(PoseStamped, '/amcl_pose', self.amcl_pose_callback, qos)
        self.get_logger().info('Subscriber created for /amcl_pose')

        # Create an action client for NavigateToPose
        self._navigate_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Timer to publish current_pose every second
        self.create_timer(1.0, self.publish_current_pose)
        self.get_logger().info('Current Pose Timer created')

        # Initialize current pose received from /amcl_pose
        self.current_pose = PoseStamped()

        # Track the state of goal
        self.goal_active = False

        # Condition to wait for goal completion
        self.goal_condition = Condition()

    def amcl_pose_callback(self, msg):
        # Store the received AMCL pose
        self.current_pose = msg

    def publish_current_pose(self):
        # Update timestamp and publish the stored pose to /current_pose
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        self.pose_publisher_.publish(self.current_pose)

    def send_goal(self, x, y, z, orientation_x=0.0, orientation_y=0.0, orientation_z=0.0, orientation_w=1.0):
        with self.goal_condition:
            if not self.goal_active:
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose.header.frame_id = 'map'
                goal_msg.pose.pose.position.x = x
                goal_msg.pose.pose.position.y = y
                goal_msg.pose.pose.position.z = z
                # Set the orientation
                goal_msg.pose.pose.orientation.x = orientation_x
                goal_msg.pose.pose.orientation.y = orientation_y
                goal_msg.pose.pose.orientation.z = orientation_z
                goal_msg.pose.pose.orientation.w = orientation_w

                self.get_logger().info('Sending goal request...')
                self._send_goal_future = self._navigate_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
                self._send_goal_future.add_done_callback(self.goal_response_callback)

                self.goal_active = True

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            with self.goal_condition:
                self.goal_active = False
                self.goal_condition.notify_all()
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal status: {result}')
        with self.goal_condition:
            self.goal_active = False
            self.get_logger().info('Goal reached successfully')
            self.goal_condition.notify_all()

    def feedback_callback(self, feedback_msg):
        # self.get_logger().info(f'Received feedback: {feedback_msg.feedback}')
        return

def main(args=None):
    rclpy.init(args=args)
    node = Nav2Commander()
    node.get_logger().info('Node initialized')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info('Node shutdown')

if __name__ == '__main__':
    main()