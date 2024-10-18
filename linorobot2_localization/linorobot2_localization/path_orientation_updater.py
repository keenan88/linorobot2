#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Quaternion
import math
import numpy as np

class PathOrientationUpdater(Node):
    def __init__(self):
        super().__init__('path_orientation_updater')
        self.path_sub = self.create_subscription(Path, '/nav2/plan', self.path_callback, 10)
        self.updated_path_pub = self.create_publisher(Path, '/nav2/plan_with_orientations', 10)
        self.get_logger().info("Path orientation updater node has been started.")

    def path_callback(self, path_msg):
        # Process the path and calculate orientations
        updated_path = self.process_path_orientations(path_msg)
        # Publish the updated path
        self.updated_path_pub.publish(updated_path)

    def process_path_orientations(self, path_msg):
        updated_path = Path()
        updated_path.header = path_msg.header
        updated_path.poses = path_msg.poses

        # Loop through the path and update orientations
        for i in range(len(path_msg.poses) - 1):
            # Get current pose and next pose to calculate yaw
            current_pose = path_msg.poses[i].pose
            next_pose = path_msg.poses[i+1].pose

            # Calculate the yaw angle between the current and next position
            delta_x = next_pose.position.x - current_pose.position.x
            delta_y = next_pose.position.y - current_pose.position.y
            yaw = math.atan2(delta_y, delta_x)

            updated_path.poses[i].pose.orientation = Quaternion(
                x = 0.0,
                y = 0.0,
                z = np.sin(yaw * 0.5),
                w = np.cos(yaw * 0.5)
            )

        # Optionally, set the orientation of the last pose to match the second last pose
        if len(path_msg.poses) > 1:
            updated_path.poses[-1].pose.orientation = updated_path.poses[-2].pose.orientation

        return updated_path

def main(args=None):
    rclpy.init(args=args)
    path_orientation_updater = PathOrientationUpdater()
    rclpy.spin(path_orientation_updater)
    path_orientation_updater.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
