#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from linorobot2_interfaces.srv import GoToNamedPose
import numpy as np
from geometry_msgs.msg import Quaternion

class NamedPoseNavigator(Node):

    def __init__(self):
        super().__init__('named_pose_navigator')

        self.declare_parameter('named_poses_file', '/home/humble_ws/src/linorobot2_navigation/config/named_poses.yaml')
        named_poses_file = self.get_parameter('named_poses_file').value
        self.named_poses = self.load_named_poses(named_poses_file)

        self.srv = self.create_service(GoToNamedPose, 'navigate_to_named_pose', self.navigate_to_pose_callback)

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def load_named_poses(self, file_path):
        with open(file_path, 'r') as file:
            try:
                return yaml.safe_load(file)['named_poses']
            except yaml.YAMLError as e:
                self.get_logger().error(f"Error reading named poses file: {e}")
                return {}
            
    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = np.sin(yaw/2)
        q.w = np.cos(yaw/2)
        
        return q


    def navigate_to_pose_callback(self, request, response):
        pose_name = request.pos_name
        if pose_name in self.named_poses:
            pose_data = self.named_poses[pose_name]
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = pose_data['x']
            goal_pose.pose.position.y = pose_data['y']

            goal_pose.pose.orientation = self.yaw_to_quaternion(pose_data['theta'])

            self.get_logger().info(f'Sending goal to x = {pose_data["x"]}, y = {pose_data["y"]}, yaw={pose_data["theta"]}')

            self.send_nav_goal(goal_pose)
            response.success = True
        else:
            response.success = False
        return response

    def send_nav_goal(self, pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.nav_to_pose_client.wait_for_server()
        

        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the server.')
            return False

        self.get_logger().info('Goal accepted, waiting for result...')

        get_result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()

        if result.status == 4:  # STATUS_SUCCEEDED (Nav2)
            self.get_logger().info('Goal reached successfully!')
            return True
        else:
            self.get_logger().warn(f'Failed to reach goal, status: {result.status}')
            return False

def main(args=None):
    rclpy.init(args=args)
    navigator = NamedPoseNavigator()
    rclpy.spin(navigator)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
