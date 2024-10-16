import rclpy
from rclpy.node import Node
from dwb_msgs.msg import LocalPlanEvaluation
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class LocalPlanEvaluator(Node):
    def __init__(self):
        super().__init__('local_plan_evaluator')
        
        # Subscriber to /nav2/evaluation topic
        self.subscription = self.create_subscription(
            LocalPlanEvaluation,
            '/nav2/evaluation',
            self.evaluation_callback,
            10
        )
        
        # Publisher to RViz for visualizing trajectories
        self.marker_publisher = self.create_publisher(MarkerArray, '/local_plan_evaluation_markers', 10)
        self.get_logger().info("Subscribed to /nav2/evaluation topic")

        self.id = 0
    
    def evaluation_callback(self, msg: LocalPlanEvaluation):
        self.get_logger().info(f"Received LocalPlanEvaluation: best_index = {msg.best_index}, worst_index = {msg.worst_index}")


        self.get_logger().info(f"Best plan: {msg.twists[msg.best_index].traj.velocity.x, msg.twists[msg.best_index].traj.velocity.y, msg.twists[msg.best_index].traj.velocity.theta}")
        for score in msg.twists[msg.best_index].scores:
            self.get_logger().info(f"{score.name[0:8]}: {round(score.raw_score, 2)} * {round(100*score.scale, 1)} = {round(100*score.raw_score * score.scale, 1)}")
        self.get_logger().info(f"                       {round(100*msg.twists[msg.best_index].total, 2)}")
        self.get_logger().info("\n")

        marker_array = MarkerArray()

        # Iterate through the evaluated trajectories
        for idx, traj in enumerate(msg.twists):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f"trajectory_{idx}"
            
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.01  # Line width
            marker.color.a = 0.1  # Transparency
            
            # Color based on the evaluation index
            if idx == msg.best_index:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0  # Best trajectory in green
                marker.scale.x = 0.0025
                marker.color.a = 1.0  # Transparency
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0  # Others in red
            
            # Create marker points for each pose in the trajectory
            for pose in traj.traj.poses:
                point = Point()
                point.x = pose.x
                point.y = pose.y
                point.z = 0.0
                marker.points.append(point)

            if idx == msg.best_index:

                marker.id = self.id
                #self.id += 1
        
                marker_array.markers.append(marker)

            # Log the trajectory score for each twist (optional)
            # self.log_trajectory_score(idx, traj)

        # Publish the MarkerArray to RViz
        self.marker_publisher.publish(marker_array)
        # self.get_logger().info(f"Published {len(marker_array.markers)} trajectory markers for visualization")
    
    def log_trajectory_score(self, idx, traj):
        """Log the evaluation scores for each trajectory."""
        self.get_logger().info(f"Trajectory {idx}:")
        for score in traj.scores:
            self.get_logger().info(f" - {score.name}: raw_score = {score.raw_score}, scale = {score.scale}")
        self.get_logger().info(f"Total score: {traj.total}")

def main(args=None):
    rclpy.init(args=args)
    node = LocalPlanEvaluator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
