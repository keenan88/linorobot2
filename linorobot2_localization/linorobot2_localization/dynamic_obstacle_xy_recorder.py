import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import csv
import os

class PointSaver(Node):
    def __init__(self):
        super().__init__('point_saver')

        # Create a subscriber to listen to /clicked_point
        self.create_subscription(PointStamped, '/clicked_point', self.point_callback, 10)

        # Define the CSV filename
        self.csv_filename = '/home/humble_ws/src/linorobot2_navigation/slam_images/0.0/' + 'clicked_points.csv'

        # Create or open the CSV file to append points
        self.file_exists = os.path.isfile(self.csv_filename)
        with open(self.csv_filename, 'a', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            if not self.file_exists:
                # Write the header if the file is new
                csv_writer.writerow(['x', 'y', 'z'])

    def point_callback(self, msg):
        """
        Callback function that gets called when a new point is clicked in RViz.
        """
        # Extract the point coordinates
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z

        # Append the point to the CSV file
        with open(self.csv_filename, 'a', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow([x, y, z])

        self.get_logger().info(f"Saved point: ({x}, {y}, {z})")

def main(args=None):
    rclpy.init(args=args)
    node = PointSaver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
