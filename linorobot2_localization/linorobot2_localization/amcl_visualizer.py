import rclpy
from rclpy.node import Node
from nav2_msgs.msg import ParticleCloud
from geometry_msgs.msg import PoseArray, Pose
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class ParticleCloudToPoseArray(Node):
    def __init__(self):
        super().__init__('particle_cloud_to_pose_array')
        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10  # Change this to the desired depth
        )

        self.pose_pub = self.create_publisher(PoseArray, '/particle_cloud_pose_array', qos_best_effort)
        self.particle_sub = self.create_subscription(ParticleCloud, '/nav2/particle_cloud', self.particle_callback, qos_best_effort)

    def particle_callback(self, particle_cloud_msg):
        pose_array = PoseArray()
        pose_array.header = particle_cloud_msg.header
        for particle in particle_cloud_msg.particles:
            pose = Pose()
            pose.position = particle.pose.position
            pose.orientation = particle.pose.orientation
            pose_array.poses.append(pose)
        self.pose_pub.publish(pose_array)

def main(args=None):
    rclpy.init(args=args)
    node = ParticleCloudToPoseArray()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
