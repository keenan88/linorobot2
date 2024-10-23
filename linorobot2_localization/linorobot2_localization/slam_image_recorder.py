import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import tf2_ros
import cv2
import os
import csv
import json

class MultiCameraListener(Node):
    def __init__(self):
        super().__init__('multi_camera_listener')

        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.depth_img_msgs = {'front': None, 'left': None, 'rear': None, 'right': None}
        self.color_img_msgs = {'front': None, 'left': None, 'rear': None, 'right': None}

        # Create subscribers for the four depth image topics
        self.right_cam_depth_sub = self.create_subscription(Image, '/right_rs/right_rs/depth/image_raw', self.depth_msg_cb, 10)
        self.left_cam_depth_sub = self.create_subscription(Image, '/left_rs/left_rs/depth/image_raw', self.depth_msg_cb, 10)
        self.front_cam_depth_sub = self.create_subscription(Image, '/front_rs/front_rs/depth/image_raw', self.depth_msg_cb, 10)
        self.rear_cam_depth_sub = self.create_subscription(Image, '/rear_rs/rear_rs/depth/image_raw', self.depth_msg_cb, 10)

        self.right_cam_color_sub = self.create_subscription(Image, '/right_rs/right_rs/color/image_raw', self.color_msg_cb, 10)
        self.left_cam_color_sub = self.create_subscription(Image, '/left_rs/left_rs/color/image_raw', self.color_msg_cb, 10)
        self.front_cam_color_sub = self.create_subscription(Image, '/front_rs/front_rs/color/image_raw', self.color_msg_cb, 10)
        self.rear_cam_color_sub = self.create_subscription(Image, '/rear_rs/rear_rs/color/image_raw', self.color_msg_cb, 10)

        self.right_cam_intrinsics_sub = self.create_subscription(CameraInfo, '/right_rs/right_rs/depth/camera_info', self.intrinsics_cb, 10)
        self.left_cam_intrinsics_sub = self.create_subscription(CameraInfo, '/left_rs/left_rs/depth/camera_info', self.intrinsics_cb, 10)
        self.front_cam_intrinsics_sub = self.create_subscription(CameraInfo, '/front_rs/front_rs/depth/camera_info', self.intrinsics_cb, 10)
        self.rear_cam_intrinsics_sub = self.create_subscription(CameraInfo, '/rear_rs/rear_rs/depth/camera_info', self.intrinsics_cb, 10)

        # TF listener for map -> base_link transform
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # CvBridge to convert ROS images to OpenCV format
        self.bridge = CvBridge()

        # Directory to save images and transforms
        t = self.get_clock().now().to_msg()
        self.save_dir = '/home/humble_ws/src/linorobot2_navigation/slam_images/' + str(t.sec + t.nanosec / 1e9) + "/"
        os.makedirs(self.save_dir, exist_ok=True)
        os.chmod(self.save_dir, 0o777)

        # Set timer to periodically query and save the transform
        self.timer = self.create_timer(0.5, self.save_transform)

    def depth_msg_cb(self, msg):
        cam_pos = msg.header.frame_id.split('_')[0]
        self.depth_img_msgs[cam_pos] = msg

    def color_msg_cb(self, msg):
        cam_pos = msg.header.frame_id.split('_')[0]
        self.color_img_msgs[cam_pos] = msg

    def intrinsics_cb(self, msg):
        camera_info_dict = {
            "header": {
                "stamp": {
                    "sec": msg.header.stamp.sec,
                    "nanosec": msg.header.stamp.nanosec
                },
                "frame_id": msg.header.frame_id
            },
            "height": msg.height,
            "width": msg.width,
            "distortion_model": msg.distortion_model,
            "d": list(msg.d),
            "k": list(msg.k),
            "r": list(msg.r),
            "p": list(msg.p),
            "binning_x": msg.binning_x,
            "binning_y": msg.binning_y,
            "roi": {
                "x_offset": msg.roi.x_offset,
                "y_offset": msg.roi.y_offset,
                "height": msg.roi.height,
                "width": msg.roi.width,
                "do_rectify": msg.roi.do_rectify
            }
        }

        path = self.save_dir + msg.header.frame_id.split('_')[0] + ".json"

        with open(path, 'w') as json_file:
            json.dump(camera_info_dict, json_file, indent=4)

        os.chmod(path, 0o777)


    def save_image(self, msg):
        """
        Callback for saving images from the subscribed camera topics.
        """

        camera_name = msg.header.frame_id.split('_')[0]  # Extracting the camera name from frame_id

        # Convert the ROS image message to an OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding if msg.encoding == '32FC1' else 'passthrough')

        # Save the image with a timestamp
        t = msg.header.stamp
        timestamp = str(t.sec + t.nanosec / 1e9)
        img_filename = os.path.join(self.save_dir, f'{camera_name}_{msg.encoding}_{timestamp}')
        img_filename += '.tiff' if msg.encoding == '32FC1' else '.png'
        cv2.imwrite(img_filename, cv_image)
        os.chmod(img_filename, 0o777)
        self.get_logger().info(f"Saved image from {camera_name} {msg.encoding} at {img_filename}")

    def save_transform(self):
        """
        Save the map -> base_link transform to a single CSV file.
        """
        try:
            image_msgs = list(self.depth_img_msgs.values()) + list(self.color_img_msgs.values())
            
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
                    
                os.chmod(transform_filename, 0o777)

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
