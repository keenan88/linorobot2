import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math


# Assumes that all laser scan topics are updated fairly quickly
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
            self.record_front_scan,
            qos_best_effort
        )
        self.subscriber_2 = self.create_subscription(
            LaserScan,
            'rear_rs/scan_left',  # Replace with your actual topic name
            self.record_rear_left_scan,
            qos_best_effort
        )
        
        self.subscriber_3 = self.create_subscription(
            LaserScan,
            '/left_rs/scan',  # Replace with your actual topic name
            self.record_left_scan,
            qos_best_effort
        )

        self.subscriber_4 = self.create_subscription(
            LaserScan,
            '/right_rs/scan',  # Replace with your actual topic name
            self.record_right_scan,
            qos_best_effort
        )

        self.subscriber_5 = self.create_subscription(
            LaserScan,
            'rear_rs/scan_right',  # Replace with your actual topic name
            self.record_rear_right_scan,
            qos_best_effort
        )

        self.publisher = self.create_publisher(LaserScan, 'scan', qos_best_effort)

        self.scans = [None] * 5

    def record_front_scan(self, msg):
        self.scans[0] = msg
        self.merge_scans()

    def record_left_scan(self, msg):
        self.scans[1] = msg
        self.merge_scans()

    def record_rear_left_scan(self, msg):
        self.scans[2] = msg
        self.merge_scans()

    def record_rear_right_scan(self, msg):
        self.scans[3] = msg
        self.merge_scans()

    def record_right_scan(self, msg):
        self.scans[4] = msg
        self.merge_scans()

    def merge_scans(self):
        if all(scan is not None for scan in self.scans):
            merged_scan = LaserScan()
            merged_scan.header.stamp = self.get_clock().now().to_msg()
            merged_scan.header.frame_id = "base_link"
            merged_scan.angle_min = -0.7850000262260437
            merged_scan.angle_max = 5.49499997377

            merged_scan.angle_increment = min(scan.angle_increment for scan in self.scans)
            merged_scan.range_min = min(scan.range_min for scan in self.scans)
            merged_scan.range_max = max(scan.range_max for scan in self.scans)

            all_ranges = []
            all_intensities = []
            for scan in self.scans:
                all_ranges += scan.ranges
                #all_intensities += scan.intensities
                # print(scan.ranges)
                # for r, i in zip(scan.ranges, scan.intensities):
                #     print(r, i)
                #     if not math.isinf(r):
                #         all_ranges.append(r)
                #         all_intensities.append(i)

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
