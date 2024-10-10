#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rmf_fleet_msgs.msg import RobotState, PathRequest
from geometry_msgs.msg import PoseStamped
import math
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses


# Intended to run on each individual robot

MODE_IDLE=0
MODE_CHARGING=1
MODE_MOVING=2
MODE_PAUSED=3
MODE_WAITING=4
MODE_EMERGENCY=5
MODE_GOING_HOME=6
MODE_DOCKING=7
MODE_ADAPTER_ERROR=8
MODE_CLEANING=9

class Linorobot2RMF(Node):

    def __init__(self):
        super().__init__('linorobot2_rmf')

        # TODO - change robot naming to read from a yaml file
        self.fleet_name = 'tinyRobot'
        self.robot_name = 'tinyRobot1'
        self.seq = 0

        self.robot_state = RobotState()
        self.robot_state.battery_percent = 100.0 # TODO - implement hardware checking for battery percent
        self.robot_state.location.level_name = "L1" # TODO - implement level updating later, if need be
        self.robot_state.location.index = 0
        self.robot_state.name = self.robot_name

        # TODO - add a bridge to bridge in and out robot_state and robot_path_requests from queen DOMAIN ID to individual robot DOMAIN ID

        self.rmf_robot_state_publisher = self.create_publisher(RobotState, 'robot_state', 10)

        self.rmf_path_request_subscription = self.create_subscription(
            PathRequest,
            'robot_path_requests',
            self.execute_path,
            10
        )

        self.odom_subscription = self.create_subscription( # Assumed that robot is operating in its own ros domain ID, so there is only 1 odom topic
            Odometry,
            '/nav2/odom',
            self.record_odometry,
            10
        )

        self._action_client = ActionClient(
            self, NavigateThroughPoses, '/nav2/navigate_through_poses')

        self.state_timer = self.create_timer(0.1, self.publish_state)

    def record_odometry(self, odom_msg):

        self.robot_state.location.t = odom_msg.header.stamp
        self.robot_state.location.x = odom_msg.pose.pose.position.x
        self.robot_state.location.y = odom_msg.pose.pose.position.y

        w = odom_msg.pose.pose.orientation.w
        x = odom_msg.pose.pose.orientation.x
        y = odom_msg.pose.pose.orientation.y
        z = odom_msg.pose.pose.orientation.z
        self.robot_state.location.yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def send_navigate_through_poses_goal(self, poses):
        """
        Send a NavigateThroughPoses action goal to the nav2 action server.
        """
        self._action_client.wait_for_server()

        # Create the NavigateThroughPoses Goal
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        self.get_logger().info(f'Sending {len(poses)} poses to Nav2: ')

        for pos in poses:
            self.get_logger().info(f'Goal: {round(pos.pose.position.x, 2)}, {round(pos.pose.position.y, 2)} ')


        # Send goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Callback to handle goal acceptance or rejection.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('NavigateThroughPoses goal was rejected.')
            return

        self.get_logger().info('NavigateThroughPoses goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Feedback callback to handle feedback from the NavigateThroughPoses action.
        """
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Position: {round(feedback.current_pose.pose.position.x, 2)}, {round(feedback.current_pose.pose.position.y, 2)} ')

    def get_result_callback(self, future):
        """
        Callback to handle the result of the NavigateThroughPoses action.
        """
        result = future.result().result
        if result:
            self.get_logger().info('NavigateThroughPoses succeeded!')
        else:
            self.get_logger().info('NavigateThroughPoses failed.')

    def execute_path(self, path_request):

        if path_request.fleet_name == self.fleet_name:
            if path_request.robot_name == self.robot_name:

                print("Executing path")
                
                self.robot_state.path = path_request.path

                self.robot_state.task_id = path_request.task_id

                self.robot_state.mode.mode_request_id = MODE_MOVING

                poses = []

                for waypoint in path_request.path:
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'  # Assuming map frame, adjust if necessary
                    pose.header.stamp = self.get_clock().now().to_msg()

                    # Assign position and orientation
                    pose.pose.position.x = waypoint.x
                    pose.pose.position.y = waypoint.y
                    pose.pose.orientation.x = 0.0
                    pose.pose.orientation.y = 0.0
                    pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
                    pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)


                    poses.append(pose)

                self.send_navigate_through_poses_goal(poses)


    
    def publish_state(self):

        self.robot_state.seq = self.seq

        self.rmf_robot_state_publisher.publish(self.robot_state)

        self.seq += 1

        # print("Published state")





        

    

def main(args=None):
    rclpy.init(args=args)
    linorobot2_rmf = Linorobot2RMF()
    rclpy.spin(linorobot2_rmf)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
