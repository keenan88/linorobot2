import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose, Quaternion
from sensor_msgs.msg import JointState
import math
import numpy as np

class MecanumStateEstimator(Node):
    def __init__(self):
        super().__init__('mecanum_state_estimator')

        self.wheel_radius = 0.0762
        self.robot_width = 0.3871
        self.robot_length = 0.51714
        
        self.x = 0.0 
        self.y = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vy = 0.0
        self.vtheta = 0.0

        # Subscriber for joint state (wheel velocities)
        self.subscription = self.create_subscription(
            JointState,
            'wheel_joint_states',
            self.joint_state_callback,
            10
        )
        
        self.odometry_publisher = self.create_publisher(Odometry, '/odom/wheels', 10)

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.publish_odometry)
        
        # Last time for integration
        self.last_time = self.get_clock().now()

    def joint_state_callback(self, msg):
        # Assuming joint names are ordered: front_left, front_right, back_left, back_right
        wheels_rad_vels = np.array(msg.velocity[0:4])

        
        # Filter out occasional Nan's and Inf's from Isaacsim joint state publisher
        if not np.any(np.isnan(wheels_rad_vels)):
            if not np.any(np.isinf(wheels_rad_vels)):

                # Compute velocities in the robot frame
                vxytheta = self.inverse_kinematics(wheels_rad_vels)
                self.vx, self.vy, self.vtheta = vxytheta

                # Integrate velocities to update position
                current_time = self.get_clock().now()
                dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds
                self.last_time = current_time

                # Update the robot's position based on the velocities
                delta_x = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
                delta_y = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
                delta_theta = self.vtheta * dt

                self.x += delta_x
                self.y += delta_y
                self.theta += delta_theta

    def inverse_kinematics(self, wheels_rad_vels):
        """
        Given wheel velocities [fl, fr, bl, br], compute vx, vy, vtheta in the robot frame.
        """
        fl, fr, bl, br = wheels_rad_vels

        # Mecanum inverse kinematics
        l = self.robot_length
        w = self.robot_width
        r = self.wheel_radius

        # Wheel velocity matrix
        A = np.array([[1, -1, -(l+w)],
                      [1, 1, (l+w)],
                      [1, 1, -(l+w)],
                      [1, -1, (l+w)]])
        
        # Robot velocity matrix
        B = np.array([fl, fr, bl, br])
        
        pseudo_inv = np.linalg.inv(np.matmul(A.T, A))
        pseudo_inv = np.matmul(pseudo_inv, A.T)
        vxytheta = np.matmul(pseudo_inv, B) * r

        return vxytheta  # [vx, vy, vtheta]

    def quaternion_from_euler_0_0(self, theta):
        # Convert Euler angles (0, 0, theta) to quaternion
        # theta is the yaw angle (rotation around the z-axis)

        # Calculate the quaternion components
        cy = np.cos(theta * 0.5)  # cos(theta/2)
        sy = np.sin(theta * 0.5)  # sin(theta/2)

        # Quaternion components (x, y, z, w)
        q = np.array([0.0, 0.0, sy, cy])  # [x, y, z, w]

        return q

    def publish_odometry(self):
        # Publish the odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set the position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Set the orientation from theta
        quat = self.quaternion_from_euler_0_0(self.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]


        # Set the velocity
        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.angular.z = self.vtheta

        # Publish the odometry message
        self.odometry_publisher.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    mecanum_state_estimator = MecanumStateEstimator()
    rclpy.spin(mecanum_state_estimator)
    mecanum_state_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
