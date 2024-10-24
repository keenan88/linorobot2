#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import csv
import os
from rosgraph_msgs.msg import Clock



class MarkerPublisher(Node):
    def __init__(self, csv_file):
        super().__init__('detected_dynamic_obstacles_publisher')
        
        # Publisher for MarkerArray
        self.publisher_ = self.create_publisher(MarkerArray, 'detected_dynamic_obstacles', 10)

        # Read CSV data
        self.data = self.read_csv_data(csv_file)

        # Timer to periodically publish markers
        self.clock_sub = self.create_subscription(Clock, '/clock', self.publish_markers, 10)

        self.marker_array = MarkerArray()
        self.counter = 0

    def read_csv_data(self, csv_file):
        """ Reads the CSV data into a list of tuples (x, y, image_filename). """
        points = []
        with open(csv_file, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                if len(row) >= 2:
                    t = float(row[0])
                    x = float(row[1])
                    y = float(row[2])
                    filename = row[3]
                    points.append((t, x, y, filename))
        return points

    def publish_markers(self, msg: Clock):
        """ Publish a set of markers corresponding to the CSV data points. """
        self.marker_array.markers.clear()

        time_s = msg.clock.sec + msg.clock.nanosec / 1e9
        
        for i, (t, x, y, filename) in enumerate(self.data):

            self.get_logger().info(f"{t, time_s, abs(t - time_s)}")

            if abs(t - time_s) < 0.1:

                marker = Marker()
                marker.header.frame_id = "map"  # Assuming map frame
                marker.header.stamp = msg.clock

                # Marker ID must be unique
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                # Position of the marker
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.1

                # Orientation (identity quaternion)
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0

                # Marker scale
                marker.scale.x = 0.2  # Adjust the size as needed
                marker.scale.y = 0.2
                marker.scale.z = 0.2

                # /home/yolo/marked/right/1004.100024313_0.png

                cam_pos = filename.split("/")[4]
                if cam_pos == 'front':

                    # Marker color (RGBA)
                    marker.color.r = 1.0
                    marker.color.g = 0.0  # Green color
                    marker.color.b = 0.0
                    marker.color.a = 1.0  # Fully opaque

                if cam_pos == 'left':

                    # Marker color (RGBA)
                    marker.color.r = 1.0
                    marker.color.g = 0.65  # Green color
                    marker.color.b = 0.0
                    marker.color.a = 1.0  # Fully opaque

                if cam_pos == 'rear':

                    # Marker color (RGBA)
                    marker.color.r = 1.0
                    marker.color.g = 1.0  # Green color
                    marker.color.b = 0.0
                    marker.color.a = 1.0  # Fully opaque

                if cam_pos == 'right':

                    # Marker color (RGBA)
                    marker.color.r = 0.0
                    marker.color.g = 1.0  # Green color
                    marker.color.b = 0.0
                    marker.color.a = 1.0  # Fully opaque

                else:

                    marker.color.a = 1.0

                # Add marker to the array
                self.marker_array.markers.append(marker)
        
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.marker_array.markers.insert(0, marker)

        # Publish the marker array
        self.publisher_.publish(self.marker_array)

def main(args=None):
    rclpy.init(args=args)
    
    # Provide the path to your CSV file here
    csv_file = '/home/yolo/obstacle_coords.csv'
    
    if not os.path.exists(csv_file):
        print(f"CSV file '{csv_file}' does not exist. Please provide a valid file.")
        return

    # Initialize the node
    marker_publisher = MarkerPublisher(csv_file)

    # Keep the node running until interrupted
    rclpy.spin(marker_publisher)

    # Shutdown
    marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
