import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf2_ros
from geometry_msgs.msg import TransformStamped
import cv2
import os
import csv

class MultiCameraListener(Node):
    def __init__(self):
        super().__init__('multi_camera_listener')

        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # Image topics
        self.right_cam_topic = '/right_rs/right_rs/depth/image_rect_raw'
        self.left_cam_topic = '/left_rs/left_rs/depth/image_rect_raw'
        self.front_cam_topic = '/front_rs/front_rs/depth/image_rect_raw'
        self.rear_cam_topic = '/rear_rs/rear_rs/depth/image_rect_raw'

        self.front_image_msg = None
        self.left_image_msg = None
        self.rear_image_msg = None
        self.right_image_msg = None

        # Create subscribers for the four depth image topics
        self.right_cam_sub = self.create_subscription(Image, self.right_cam_topic, self.front_cb, 10)
        self.left_cam_sub = self.create_subscription(Image, self.left_cam_topic, self.left_cb, 10)
        self.front_cam_sub = self.create_subscription(Image, self.front_cam_topic, self.rear_cb, 10)
        self.rear_cam_sub = self.create_subscription(Image, self.rear_cam_topic, self.right_cb, 10)

        # TF listener for map -> base_link transform
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # CvBridge to convert ROS images to OpenCV format
        self.bridge = CvBridge()

        # Directory to save images and transforms
        t = self.get_clock().now().to_msg()
        print(t)
        self.save_dir = '/home/humble_ws/src/linorobot2_navigation/slam_images/' + str(t.sec + t.nanosec / 1e9)
        os.makedirs(self.save_dir, exist_ok=True)

        # Set timer to periodically query and save the transform
        self.timer = self.create_timer(1.0, self.save_transform)

    def front_cb(self, msg):
        self.front_image_msg = msg

    def left_cb(self, msg):
        self.left_image_msg = msg

    def rear_cb(self, msg):
        self.rear_image_msg = msg

    def right_cb(self, msg):
        self.right_image_msg = msg

    def save_image(self, msg):
        """
        Callback for saving images from the subscribed camera topics.
        """

        camera_name = msg.header.frame_id.split('_')[0]  # Extracting the camera name from frame_id

        # Convert the ROS image message to an OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Save the image with a timestamp
        t = msg.header.stamp
        timestamp = str(t.sec + t.nanosec / 1e9)
        img_filename = os.path.join(self.save_dir, f'{camera_name}_{timestamp}.png')
        cv2.imwrite(img_filename, cv_image)
        self.get_logger().info(f"Saved image from {camera_name} at {img_filename}")

    import csv

    def save_transform(self):
        """
        Save the map -> base_link transform to a single CSV file.
        """
        try:
            image_msgs = [self.front_image_msg, self.left_image_msg, self.rear_image_msg, self.right_image_msg]
            if None not in image_msgs:

                # Get the transform from map to base_link
                transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

                # Get the current timestamp
                t = transform.header.stamp

                timestamp = str(t.sec + t.nanosec / 1e9)

                for image_msg in image_msgs:
                    self.save_image(image_msg)

                # Prepare transform data for saving
                translation = transform.transform.translation
                rotation = transform.transform.rotation

                # Define the CSV filename (will create if not exists)
                transform_filename = os.path.join(self.save_dir, 'transforms.csv')

                # Write or append the transform to the CSV file
                file_exists = os.path.isfile(transform_filename)

                with open(transform_filename, 'a', newline='') as csvfile:
                    csv_writer = csv.writer(csvfile)

                    # If file does not exist, write the header
                    if not file_exists:
                        csv_writer.writerow(['timestamp', 'translation_x', 'translation_y', 'translation_z',
                                            'rotation_x', 'rotation_y', 'rotation_z', 'rotation_w'])

                    # Write the transform data as a new row
                    csv_writer.writerow([timestamp, translation.x, translation.y, translation.z,
                                        rotation.x, rotation.y, rotation.z, rotation.w])

                self.get_logger().info(f"Saved transform to {transform_filename}")
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"Could not get transform: {ex}")


def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraListener()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
