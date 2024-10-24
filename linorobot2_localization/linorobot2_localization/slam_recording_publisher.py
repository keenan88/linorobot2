import rclpy
from rclpy.node import Node
import csv
import os
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32
from rosgraph_msgs.msg import Clock
import tf2_ros
from cv_bridge import CvBridge
import cv2, math

class TransformImagePublisher(Node):
    def __init__(self):
        super().__init__('transform_image_publisher')

        # Initialize CvBridge to convert images
        self.bridge = CvBridge()

        # Directory where images are saved
        self.image_dir = '/home/yolo/slam_images/0.0/'

        # Load transforms from CSV
        self.transforms = self.load_transforms()

        # Create publishers for the images and transforms
        self.right_cam_pub = self.create_publisher(Image, '/right_rs/right_rs/color/image_raw', 10)
        self.left_cam_pub = self.create_publisher(Image, '/left_rs/left_rs/color/image_raw', 10)
        self.front_cam_pub = self.create_publisher(Image, '/front_rs/front_rs/color/image_raw', 10)
        self.rear_cam_pub = self.create_publisher(Image, '/rear_rs/rear_rs/color/image_raw', 10)

        # TF broadcaster for transforms
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timestamp_publisher = self.create_publisher(Clock, '/clock', 10)

        # Publish each transform and the nearest images every second
        self.slam_step_subscriber = self.create_subscription(Int32, '/slam_step', self.update_slam_step, 10)

    def update_slam_step(self, msg):

        self.publish_transform_and_images(msg.data)

        

    def load_transforms(self):
        """
        Load transforms from the CSV file.
        """
        transform_filename = os.path.join(self.image_dir, 'transforms.csv')
        transforms = []
        with open(transform_filename, 'r') as csvfile:
            csv_reader = csv.DictReader(csvfile)
            for row in csv_reader:
                transform = {
                    'timestamp': float(row['t']),
                    'translation_x': float(row['x']),
                    'translation_y': float(row['y']),
                    'translation_z': 0.0,
                    'rotation_x': 0.0,
                    'rotation_y': 0.0,
                    'rotation_z': math.sin(float(row['yaw']) / 2),
                    'rotation_w': math.cos(float(row['yaw']) / 2),
                    'depth_image_path': row['depth_image_path'],
                    'clr_image_path': row['clr_image_path']

                }
                transforms.append(transform)
        return transforms

    def publish_transform_and_images(self, idx):
        if idx >= len(self.transforms):
            self.get_logger().info("No more transforms to publish.")
            return

        transform = self.transforms[idx]
        self.publish_transform(transform)

        timestamp = transform['timestamp']
        self.publish_timestamp(timestamp)


        self.publish_image(transform['clr_image_path'])


    def publish_timestamp(self, timestamp):

        clock_msg = Clock()
        timestamp = str(timestamp)
        clock_msg.clock.sec = int(timestamp.split(".")[0])
        ns = timestamp.split(".")[1]
        while len(ns) < 9: ns += '0'
        clock_msg.clock.nanosec = int(ns)

        self.timestamp_publisher.publish(clock_msg)

    def publish_transform(self, transform):
        """
        Publish the given transform as a static transform.
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = transform['translation_x']
        t.transform.translation.y = transform['translation_y']
        t.transform.translation.z = transform['translation_z']
        t.transform.rotation.x = transform['rotation_x']
        t.transform.rotation.y = transform['rotation_y']
        t.transform.rotation.z = transform['rotation_z']
        t.transform.rotation.w = transform['rotation_w']

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Published transform for timestamp {transform['timestamp']}")

    def publish_image(self, image_path):
        # /home/yolo/slam_images/0.0/right_32FC1_1001.90002426.tiff

        cam = image_path.split("/")[5].split('_')[0]
        
        cv_image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
        image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='passthrough')
        image_msg.header.stamp = self.get_clock().now().to_msg()
        
        if cam == 'right':
            self.right_cam_pub.publish(image_msg)
        elif cam == 'left':
            self.left_cam_pub.publish(image_msg)
        elif cam == 'front':
            self.front_cam_pub.publish(image_msg)
        elif cam == 'rear':
            self.rear_cam_pub.publish(image_msg)

        self.get_logger().info(f"Published {cam} camera image from {image_path}")

def main(args=None):
    rclpy.init(args=args)
    node = TransformImagePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
