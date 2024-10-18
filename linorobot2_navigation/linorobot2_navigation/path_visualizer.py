#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray


class PathVisualizer(Node):

    def __init__(self):
        super().__init__('path_visualizer')
        self.path_sub = self.create_subscription(
            Path,
            '/nav2/plan_with_orientations',
            self.path_callback,
            10
        )
        self.marker_pub = self.create_publisher(MarkerArray, '/path_visualization', 10)
        self.get_logger().info("Path visualizer node has been started.")

    def path_callback(self, path_msg):
        marker_array = MarkerArray()

        for idx, pose in enumerate(path_msg.poses):
            # Visualize each pose as a sphere
            marker = Marker()
            marker.header.frame_id = path_msg.header.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'path_visualization'
            marker.id = idx
            marker.type = Marker.SPHERE
            marker.action = Marker.MODIFY
            marker.pose = pose.pose
            marker.scale.x = 0.025
            marker.scale.y = 0.025
            marker.scale.z = 0.025
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            # Visualize the twist (velocity arrow) at this point
            twist_marker = Marker()
            twist_marker.header.frame_id = path_msg.header.frame_id
            twist_marker.header.stamp = self.get_clock().now().to_msg()
            twist_marker.ns = 'twist_visualization'
            twist_marker.id = 1000 + idx  # Different ID space for twist markers
            twist_marker.type = Marker.ARROW
            twist_marker.action = Marker.ADD
            twist_marker.pose.position = pose.pose.position
            twist_marker.pose.orientation = pose.pose.orientation

            # Twist marker size (you can customize)
            twist_marker.scale.x = 0.1  # Length of arrow
            twist_marker.scale.y = 0.02  # Width of arrow shaft
            twist_marker.scale.z = 0.02  # Width of arrow head

            twist_marker.color.a = 1.0
            twist_marker.color.r = 1.0
            twist_marker.color.g = 0.0
            twist_marker.color.b = 0.0

            # Add markers to the array
            marker_array.markers.append(marker)
            marker_array.markers.append(twist_marker)

        # Publish the marker array
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    path_visualizer = PathVisualizer()
    rclpy.spin(path_visualizer)
    path_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
