import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import math
import numpy as np
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class MecanumStateEstimator(Node):
    def __init__(self):
        super().__init__('mecanum_state_estimator')

        self.wheel_radius_m = 0.0762
        self.wheel_seperation_width_m = 0.3871
        self.wheel_seperation_length_m = 0.51714
        
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

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.odom_base_footprint_tf = TransformStamped()
        self.odom_base_footprint_tf.header.frame_id = 'odom'
        self.odom_base_footprint_tf.child_frame_id = 'base_footprint'
        
        self.odometry_publisher = self.create_publisher(Odometry, '/nav2/odom', 10)
        
        self.last_time = -1
        self.first_time_set = False

    def joint_state_callback(self, msg):
        if not self.first_time_set:
            self.last_time = self.get_clock().now()
            self.first_time_set = True
            self.x = 0.0 
            self.y = 0.0
            self.theta = 0.0
            self.vx = 0.0
            self.vy = 0.0
            self.vtheta = 0.0
        else:
            # Assuming joint names are ordered: front_left, front_right, back_left, back_right
            wheels_rad_vels = np.array(msg.velocity[0:4])
            
            self.vx, self.vy, self.vtheta = self.inverse_kinematics(wheels_rad_vels)

            # Integrate velocities to update position
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds
            print(dt)
            self.last_time = current_time

            # Update the robot's position based on the velocities
            delta_x = (self.vx * math.cos(self.theta) - self.vy * math.sin(self.theta)) * dt
            delta_y = (self.vx * math.sin(self.theta) + self.vy * math.cos(self.theta)) * dt
            delta_theta = self.vtheta * dt

            self.x += delta_x
            self.y += delta_y
            self.theta += delta_theta

            self.publish_odometry()

    def inverse_kinematics(self, wheels_rad_vels):
        # Kinematics source: https://www.researchgate.net/publication/308570348_Inverse_kinematic_implementation_of_four-wheels_mecanum_drive_mobile_robot_using_stepper_motors
        fl, fr, bl, br = wheels_rad_vels

        l = self.wheel_seperation_length_m
        w = self.wheel_seperation_width_m
        r = self.wheel_radius_m

        A = np.array([[1, -1, -(l+w)],
                      [1, 1, (l+w)],
                      [1, 1, -(l+w)],
                      [1, -1, (l+w)]])
        
        w = np.array([fl, fr, bl, br])
        
        pseudo_inv = np.linalg.pinv(A)
        vxytheta = np.matmul(pseudo_inv, w) * r

        vxytheta[1] /= 1.2 # Scaled y velocity, based on observations in simulation
        vxytheta[2] *= 2 / 1.07 # Scaled angular velocity, based on observations in simulation


        return vxytheta

    def quaternion_from_euler_0_0(self, theta):
        cy = np.cos(theta * 0.5)
        sy = np.sin(theta * 0.5)

        q = np.array([0.0, 0.0, sy, cy])

        return q

    def publish_odometry(self):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        quat = self.quaternion_from_euler_0_0(self.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        odom_msg.twist.twist.linear.x = self.vx
        odom_msg.twist.twist.linear.y = self.vy
        odom_msg.twist.twist.angular.z = self.vtheta

        self.odometry_publisher.publish(odom_msg)

        self.odom_base_footprint_tf.transform.translation.x = self.x
        self.odom_base_footprint_tf.transform.translation.y = self.y

        self.odom_base_footprint_tf.transform.rotation.x = quat[0]
        self.odom_base_footprint_tf.transform.rotation.y = quat[1]
        self.odom_base_footprint_tf.transform.rotation.z = quat[2]
        self.odom_base_footprint_tf.transform.rotation.w = quat[3]

        self.odom_base_footprint_tf.header.stamp = self.get_clock().now().to_msg()
        self.tf_static_broadcaster.sendTransform(self.odom_base_footprint_tf)

        # asdf = TransformStamped()
        # asdf.header.frame_id = 'map'
        # asdf.child_frame_id = 'odom'
        # asdf.header.stamp = self.get_clock().now().to_msg()
        # self.tf_static_broadcaster.sendTransform(asdf)



def main(args=None):
    rclpy.init(args=args)
    mecanum_state_estimator = MecanumStateEstimator()
    rclpy.spin(mecanum_state_estimator)
    mecanum_state_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
