import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math

class LaserScanMerger(Node):
    def __init__(self):
        super().__init__('laser_scan_merger')

        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10  # Change this to the desired depth
        )


        # Subscribers for the four laser scan topics
        self.subscriber_1 = self.create_subscription(
            LaserScan,
            '/front_rs/scan',  # Replace with your actual topic name
            self.scan_callback_1,
            qos_best_effort
        )
        self.subscriber_2 = self.create_subscription(
            LaserScan,
            'rear_rs/scan',  # Replace with your actual topic name
            self.scan_callback_2,
            qos_best_effort
        )
        self.subscriber_3 = self.create_subscription(
            LaserScan,
            '/left_rs/scan',  # Replace with your actual topic name
            self.scan_callback_3,
            qos_best_effort
        )
        self.subscriber_4 = self.create_subscription(
            LaserScan,
            '/right_rs/scan',  # Replace with your actual topic name
            self.scan_callback_4,
            qos_best_effort
        )

        # Publisher for the combined scan
        self.publisher = self.create_publisher(LaserScan, 'scan', qos_best_effort)

        # Store laser scans
        self.scans = [None] * 4

    def scan_callback_1(self, msg):
        self.scans[0] = msg
        self.merge_scans()

    def scan_callback_2(self, msg):
        self.scans[1] = msg
        self.merge_scans()

    def scan_callback_3(self, msg):
        self.scans[2] = msg
        self.merge_scans()

    def scan_callback_4(self, msg):
        self.scans[3] = msg
        self.merge_scans()

    def merge_scans(self):
        if all(scan is not None for scan in self.scans):
            merged_scan = LaserScan()
            merged_scan.header.stamp = self.get_clock().now().to_msg()
            merged_scan.header.frame_id = "base_link"
            merged_scan.angle_min = min(scan.angle_min for scan in self.scans)
            merged_scan.angle_max = max(scan.angle_max for scan in self.scans)
            merged_scan.angle_increment = min(scan.angle_increment for scan in self.scans)
            merged_scan.range_min = min(scan.range_min for scan in self.scans)
            merged_scan.range_max = max(scan.range_max for scan in self.scans)

            # Merge the ranges and intensities
            all_ranges = []
            all_intensities = []
            for scan in self.scans:
                for r, i in zip(scan.ranges, scan.intensities):
                    if not math.isinf(r):
                        all_ranges.append(r)
                        all_intensities.append(i)

            merged_scan.ranges = all_ranges
            merged_scan.intensities = all_intensities
            merged_scan.time_increment = sum(scan.time_increment for scan in self.scans) / len(self.scans)

            self.publisher.publish(merged_scan)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
