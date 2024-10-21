import rclpy
from rclpy.node import Node
from rmf_ingestor_msgs.msg import IngestorRequest, IngestorState, IngestorResult
from rmf_dispenser_msgs.msg import DispenserRequest
import uuid

IDLE=0
BUSY=1
OFFLINE=2

ACKNOWLEDGED=0
SUCCESS=1
FAILED=2


class DummyIngestor(Node):
    def __init__(self):
        super().__init__('dummy_ingestor')

        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.completed_request_ids = []

        # Publisher for ingestor state
        self.state_publisher = self.create_publisher(IngestorState, 'ingestor_states', 10)
        self.state_publisher_timer = self.create_timer(2.0, self.publish_state)
        self.state = IngestorState()
        self.state.mode = IDLE # Assume no worker at ingestor, until worker updates ingestor state
        self.state.guid = 'worker_dropoff_L1'

        # Subscriber for ingestor requests
        self.request_subscription = self.create_subscription(
            IngestorRequest,
            'ingestor_requests',
            self.ingestor_request_callback,
            10
        )
        
        self.result_publisher = self.create_publisher(
            IngestorResult,
            'ingestor_results',
            10
        )

        self.dispenser_request_publisher = self.create_publisher(
            DispenserRequest,
            'dispenser_requests',
            10
        )

        self.get_logger().info("worker_dropoff_L1 Ingestor Node has started.")

    def publish_state(self):

        self.state.time = self.get_clock().now().to_msg()
        self.state_publisher.publish(self.state)

    def ingestor_request_callback(self, msg: IngestorRequest):
        self.get_logger().info(f"Received request for {msg.request_guid}")

        if msg.target_guid == self.state.guid and "delivery.dispatch" in msg.request_guid:

            if msg.request_guid not in self.completed_request_ids:

                worker_available_request = DispenserRequest()
                worker_available_request.time = self.get_clock().now().to_msg()
                worker_available_request.request_guid = "worker_is_available:" + str(uuid.uuid4())
                worker_available_request.target_guid = "dummy_dispenser"

                self.dispenser_request_publisher.publish(worker_available_request)

                self.completed_request_ids.append(msg.request_guid)

                ingestor_result = IngestorResult()
                ingestor_result.time = self.get_clock().now().to_msg()
                ingestor_result.request_guid = msg.request_guid
                ingestor_result.status = SUCCESS

                self.result_publisher.publish(ingestor_result)

                


            

            


def main(args=None):
    rclpy.init(args=args)

    dummy_ingestor = DummyIngestor()

    try:
        rclpy.spin(dummy_ingestor)
    except KeyboardInterrupt:
        pass
    finally:
        dummy_ingestor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
