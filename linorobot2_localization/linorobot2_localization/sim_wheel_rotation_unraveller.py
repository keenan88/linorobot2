import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from copy import deepcopy
import math


class WheelRotationUnraveller(Node):
    def __init__(self):
        super().__init__('sim_wheel_rotation_unraveller')

        self.wheel_state_subscriber = self.create_subscription(JointState, '/wheel_joint_states_isaacsim', self.unravel_wheel, 10)
        self.wheel_state_publisher = self.create_publisher(JointState, '/wheel_joint_states', 10)

        self.previous_angles = np.zeros(4)
        self.total_displacement = np.zeros(4)

    
    def unravel_wheel(self, wheel_state: JointState):

        # Sometimes IsaacSim Joint State Publisher Action Node publishes velocities or positions that are inf, nan, or in the 100s of rad/s.
        pos_has_NaN = np.any(np.isnan(wheel_state.position[0:4]))
        vel_has_NaN = np.any(np.isnan(wheel_state.velocity[0:4]))
        pos_has_Inf = np.any(np.isinf(wheel_state.position[0:4]))
        vel_has_Inf = np.any(np.isinf(wheel_state.velocity[0:4]))
        vel_unreasonably_high = np.any(np.abs(wheel_state.velocity[0:4]) > 30)

        if not (pos_has_NaN or vel_has_NaN or pos_has_Inf or vel_has_Inf or vel_unreasonably_high):

            current_angles = np.array(wheel_state.position[0:4])
    
            delta_angles = current_angles - self.previous_angles

            for i in range(len(current_angles)):

                if current_angles[i] > np.pi:
                    current_angles[i] -= 4 * np.pi
                elif current_angles[i] < -np.pi: 
                    current_angles[i] += 4 * np.pi

                current_angles[i] += delta_angles[i]

            self.previous_angles = current_angles

            wheel_state_unravelled = JointState()
            wheel_state_unravelled.header = wheel_state.header
            wheel_state_unravelled.position = list(current_angles)
            wheel_state_unravelled.velocity = wheel_state.velocity[0:4]
        
            self.wheel_state_publisher.publish(wheel_state_unravelled)

def main(args=None):
    rclpy.init(args=args)
    sim_wheel_rotation_unraveller = WheelRotationUnraveller()

    rclpy.spin(sim_wheel_rotation_unraveller)

    

if __name__ == '__main__':
    main()