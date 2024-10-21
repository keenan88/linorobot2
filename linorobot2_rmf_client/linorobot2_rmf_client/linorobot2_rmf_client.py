#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rmf_fleet_msgs.msg import RobotState, PathRequest
from geometry_msgs.msg import PoseStamped
import math
from tf2_ros import Buffer, TransformListener
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from geometry_msgs.msg import TransformStamped
from copy import deepcopy



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

        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])


        # TODO - change robot naming to read from a yaml file
        self.fleet_name = 'drone_fleet'
        self.robot_name = 'john_the_drone'
        self.seq = 0

        self.robot_state = RobotState()
        self.robot_state.battery_percent = 100.0 # TODO - implement hardware checking for battery percent
        self.robot_state.location.level_name = "L1" # TODO - implement level updating later, if need be
        self.robot_state.location.index = 0
        self.robot_state.name = self.robot_name
        self.robot_state.mode.mode = MODE_IDLE

        # TODO - add a bridge to bridge in and out robot_state and robot_path_requests from queen DOMAIN ID to individual robot DOMAIN ID

        self.rmf_robot_state_publisher = self.create_publisher(RobotState, 'robot_state', 10)

        self.rmf_path_request_subscription = self.create_subscription(
            PathRequest,
            'robot_path_requests',
            self.add_path_to_queue,
            10
        )

        self.execute_path_timer = self.create_timer(0.01, self.execute_path)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.01, self.get_transform)
        self.first_tf_set = False

        self._action_client = ActionClient(
            self, NavigateToPose, '/nav2/navigate_to_pose')

        self.state_timer = self.create_timer(0.01, self.publish_state)

        self.path_requests = []
        self.completed_tasks_IDs = []



    def get_transform(self):
        try:
            now = rclpy.time.Time()
            timeout = rclpy.duration.Duration(seconds=1.0)
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'map', 'base_link', now, timeout
            )

            x = transform.transform.rotation.x
            y = transform.transform.rotation.y
            z = transform.transform.rotation.z
            w = transform.transform.rotation.w

            self.robot_state.location.x = transform.transform.translation.x
            self.robot_state.location.y = transform.transform.translation.y
            self.robot_state.location.yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
            self.robot_state.location.t = transform.header.stamp
            self.first_tf_set = True

        except Exception as e:
            self.get_logger().info(f"Could not get transform: {e}")

    def send_navigate_through_poses_goal(self, pose):
        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('NavigateThroughPoses goal was rejected.')
            self.robot_state.mode.mode = MODE_IDLE
            self.get_logger().info('goal_response_callback: set to idle')
            self.robot_state.path = []
            return

        self.get_logger().info('NavigateThroughPoses goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(f'Position: {round(feedback.current_pose.pose.position.x, 2)}, {round(feedback.current_pose.pose.position.y, 2)} ')

    def get_result_callback(self, future):
        result = future.result().result

        if result: 
            self.get_logger().info('NavigateThroughPoses succeeded!')
            self.completed_tasks_IDs.append(self.robot_state.task_id)
        else: 
            self.get_logger().info('NavigateThroughPoses failed.')

        self.robot_state.mode.mode = MODE_IDLE
        self.get_logger().info('get_result_callback: set to idle')
        self.robot_state.path = []

    def execute_path(self):

        if len(self.path_requests) > 0:

            if self.robot_state.mode.mode in [MODE_IDLE, MODE_WAITING, MODE_CHARGING]:

                self.robot_state.mode.mode = MODE_MOVING
                self.get_logger().info('execute_path: set to idle')

                path_request = self.path_requests.pop(0)
                        
                self.robot_state.path = [path_request.path[1]]
                self.robot_state.task_id = path_request.task_id

                waypoint = path_request.path[1]

                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = waypoint.t

                pose.pose.position.x = waypoint.x
                pose.pose.position.y = waypoint.y
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
                pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)

                self.send_navigate_through_poses_goal(pose)

                self.get_logger().info(f"Sent:  {path_request.task_id} : \
                    {round(path_request.path[0].x, 2), round(path_request.path[0].y, 2), round(path_request.path[0].yaw, 2)} \
                    ) -> ( \
                    {round(path_request.path[1].x, 2), round(path_request.path[1].y, 2), round(path_request.path[1].yaw, 2)}"
                )

    def add_path_to_queue(self, path_request):
        if path_request.fleet_name == self.fleet_name:

            if path_request.robot_name == self.robot_name:

                if not (path_request.task_id == self.robot_state.task_id and self.robot_state.mode.mode == MODE_MOVING):

                    if path_request.task_id not in [path_request.task_id for path_request in self.path_requests]:

                        if path_request.task_id not in self.completed_tasks_IDs:

                            self.path_requests.append(path_request)

                            self.get_logger().info(f"Queued:  {path_request.task_id} : \
                                {round(path_request.path[0].x, 2), round(path_request.path[0].y, 2), round(path_request.path[0].yaw, 2)} \
                                ) -> ( \
                                {round(path_request.path[1].x, 2), round(path_request.path[1].y, 2), round(path_request.path[1].yaw, 2)}"
                            )

            
    
    def publish_state(self):

        if self.first_tf_set:

            self.robot_state.seq = self.seq

            self.rmf_robot_state_publisher.publish(self.robot_state)

            self.seq += 1
    

def main(args=None):
    rclpy.init(args=args)
    linorobot2_rmf = Linorobot2RMF()
    rclpy.spin(linorobot2_rmf)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
