# navigate_through_poses.py

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import sys

class NavigateThroughPosesClient(Node):
    def __init__(self):
        super().__init__('navigate_through_poses_client')
        self._action_client = ActionClient(self, NavigateThroughPoses, '/nav2/navigate_through_poses')

    def send_goal(self, poses):
        # Wait for the action server to be available
        self._action_client.wait_for_server()

        # Create the goal message
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        # Send the goal to the action server
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
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
        self.get_logger().info(f'Received feedback: {feedback.current_pose}')

    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().info('Goal failed.')

def main(args=None):
    rclpy.init(args=args)

    node = NavigateThroughPosesClient()

    # Define a series of PoseStamped messages (example)
    poses = []

    pose1 = PoseStamped()
    pose1.header.frame_id = 'map'
    pose1.header.stamp = node.get_clock().now().to_msg()
    pose1.pose.position.x = 6.0
    pose1.pose.position.y = -1.0
    pose1.pose.orientation.w = 1.0

    pose2 = PoseStamped()
    pose2.header.frame_id = 'map'
    pose2.header.stamp = node.get_clock().now().to_msg()
    pose2.pose.position.x = 5.0
    pose2.pose.position.y = -2.0
    pose2.pose.orientation.w = 1.0

    pose3 = PoseStamped()
    pose3.header.frame_id = 'map'
    pose3.header.stamp = node.get_clock().now().to_msg()
    pose3.pose.position.x = 3.0
    pose3.pose.position.y = 3.0
    pose3.pose.orientation.w = 1.0

    # Add the poses to the list
    poses.append(pose1)
    poses.append(pose2)
    poses.append(pose3)

    # Send goal to navigate through poses
    node.send_goal(poses)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Navigation canceled by user.')

    # Shutdown
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
