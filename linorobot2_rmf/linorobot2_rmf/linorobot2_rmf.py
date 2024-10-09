#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rmf_fleet_msgs.msg import RobotState, PathRequest
import math

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
        self.fleet_name = 'fleet1'
        self.robot_name = 'george'

        self.robot_state = RobotState()
        self.robot_state.battery_percent = 1.0 # TODO - implement hardware checking for battery percent
        self.robot_state.location.level_name = "L1" # TODO - implement level updating later, if need be
        self.robot_state.location.index = 0

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

    def execute_path(self, path_request):

        if path_request.fleet_name == self.fleet_name:
            if path_request.robot_name == self.robot_name:
                
                self.robot_state.path = path_request.path

                self.robot_state.task_id = path_request.task_id

                self.robot_state.mode_request_id = MODE_MOVING

                # TODO - make call to move_through_poses nav2 action server (async?)


    
    def publish_state(self):

        self.rmf_robot_state_publisher.publish(self.robot_state)





        

    

def main(args=None):
    rclpy.init(args=args)
    linorobot2_rmf = Linorobot2RMF()
    rclpy.spin(linorobot2_rmf)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
