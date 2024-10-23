import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2
from cv_bridge import CvBridge
import numpy as np
import struct

class DepthListener(Node):
    def __init__(self):
        super().__init__('depth_listener')
        
        # Initialize a CvBridge to convert ROS images to OpenCV images
        self.bridge = CvBridge()
        
        # Initialize variables to store camera intrinsic parameters
        self.camera_info_received = False
        self.fx = 0
        self.fy = 0
        self.cx = 0
        self.cy = 0
        
        # Create a subscriber to the depth image topic
        self.subscription = self.create_subscription(
            Image,
            '/front_rs/front_rs/depth/image_raw',
            self.listener_callback,
            10)
        
        # Create a subscriber to the camera info topic
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/front_rs/front_rs/depth/camera_info',
            self.camera_info_callback,
            10)
        
    def camera_info_callback(self, msg):
        # Extract the camera intrinsic parameters
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.camera_info_received = True
        self.get_logger().info(f"Camera info received: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")
    
    def listener_callback(self, msg):
        # Wait until camera info has been received
        if not self.camera_info_received:
            self.get_logger().warning('Waiting for camera info...')
            return
        
        # Convert the ROS Image message to a NumPy array using cv_bridge
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return
        
        # Check the shape of the depth image
        if depth_image is not None:
            self.get_logger().info(f"Depth image shape: {depth_image.shape}")
            
            # Define pixel coordinates of interest (center and corners)
            height, width = depth_image.shape
            center_pixel = (height // 2, width // 2)
            corners = [(0, 0), (0, width-1), (height-1, 0), (height-1, width-1)]
            
            # Compute and print 3D coordinates for center pixel
            depth_center = depth_image[center_pixel]
            x_center = (center_pixel[1] - self.cx) * depth_center / self.fx
            y_center = (center_pixel[0] - self.cy) * depth_center / self.fy
            self.get_logger().info(f"Center pixel 3D coordinates: X={x_center:.2f} m, Y={y_center:.2f} m, Z={depth_center:.2f} m")
            
            # Compute and print 3D coordinates for the corner pixels
            for corner in corners:
                depth_value = depth_image[corner]
                x = (corner[1] - self.cx) * depth_value / self.fx
                y = (corner[0] - self.cy) * depth_value / self.fy
                self.get_logger().info(f"Pixel {corner} 3D coordinates: X={x:.2f} m, Y={y:.2f} m, Z={depth_value:.2f} m")
        else:
            self.get_logger().warning('Empty depth image received')

def main(args=None):
    rclpy.init(args=args)
    node = DepthListener()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
