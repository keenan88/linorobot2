import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class MecanumRobotController(Node):
    def __init__(self):
        super().__init__('mecanum_robot_controller')

        self.wheel_radius_m = 0.0762

        # Subscriber for cmd_vel
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.joint_state_publisher = self.create_publisher(
            JointState,
            'wheel_joints_command',
            10
        )

        self.wheel_names = [
            'base_link_to_front_left_base_link', 
            'base_link_to_front_right_base_link', 
            'base_link_to_rear_left_base_link', 
            'base_link_to_rear_right_base_link'
        ]

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        v_fl = linear_x - linear_y - (angular_z * 0.5)  # Front Left
        v_fr = linear_x + linear_y + (angular_z * 0.5)  # Front Right
        v_rl = linear_x + linear_y - (angular_z * 0.5)  # Rear Left
        v_rr = linear_x - linear_y + (angular_z * 0.5)  # Rear Right

        w_fl = v_fl / self.wheel_radius_m
        w_fr = v_fr / self.wheel_radius_m
        w_rl = v_rl / self.wheel_radius_m
        w_rr = v_rr / self.wheel_radius_m

        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.wheel_names
        joint_state.velocity = [w_fl, w_fr, w_rl, w_rr]

        self.joint_state_publisher.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    mecanum_robot_controller = MecanumRobotController()
    
    rclpy.spin(mecanum_robot_controller)

    # Cleanup
    mecanum_robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
