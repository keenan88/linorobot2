import rclpy
from rclpy.node import Node
from rmf_dispenser_msgs.msg import DispenserRequest, DispenserState, DispenserResult

IDLE=0
BUSY=1
OFFLINE=2

ACKNOWLEDGED=0
SUCCESS=1
FAILED=2


class DummyDispenser(Node):
    def __init__(self):
        super().__init__('dummy_dispenser')

        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # Publisher for dispenser state
        self.state_publisher = self.create_publisher(DispenserState, 'dispenser_states', 10)
        self.state_publisher_timer = self.create_timer(2.0, self.publish_state)
        self.state = DispenserState()
        self.state.mode = IDLE # Assume no worker at dispenser, until worker updates dispenser state
        self.state.guid = 'worker_pickup_R1'

        self.is_worker_available = True

        # Subscriber for dispenser requests
        self.request_subscription = self.create_subscription(
            DispenserRequest,
            'dispenser_requests',
            self.dispenser_request_callback,
            10
        )
        
        self.dispense_result_publisher = self.create_publisher(
            DispenserResult,
            'dispenser_results',
            10
        )

        self.current_task_guid = ""
        self.completed_task_guids = []

        self.get_logger().info("worker_pickup_R1 Dispenser Node has started.")

    def publish_state(self):

        self.state.time = self.get_clock().now().to_msg()
        self.state_publisher.publish(self.state)

    def dispenser_request_callback(self, msg: DispenserRequest):
        self.get_logger().info(f"Received request for {msg.request_guid}")

        if msg.target_guid == self.state.guid:

            if msg.request_guid not in self.completed_task_guids:

                if msg.request_guid != self.current_task_guid:

                    dispense_result = DispenserResult()
                    dispense_result.time = self.get_clock().now().to_msg()
                    dispense_result.request_guid = msg.request_guid

                    if "delivery.dispatch" in msg.request_guid and self.is_worker_available:

                        self.current_task_guid = msg.request_guid

                        self.state.mode = BUSY

                        # TODO - add blocking call to action server that makes drone pickup worker

                        # dispense_result.source_guid = 
                        dispense_result.status = SUCCESS # TODO - set to failed, if pickup fails
                        # is_worker_available is commented out while debugging with just drone
                        # self.is_worker_available = False # TODO - update is_worker_available based on whether or not worker was successfully picked up by drone

                        self.state.mode = IDLE

                    elif "worker_is_available" in msg.request_guid:

                        self.current_task_guid = msg.request_guid

                        self.is_worker_available = True
                        
                        # dispense_result.source_guid = 
                        dispense_result.status = SUCCESS

                    self.dispense_result_publisher.publish(dispense_result)
                    self.current_task_guid = ""
                    self.completed_task_guids.append(msg.request_guid)
                


def main(args=None):
    rclpy.init(args=args)

    dummy_dispenser = DummyDispenser()

    try:
        rclpy.spin(dummy_dispenser)
    except KeyboardInterrupt:
        pass
    finally:
        dummy_dispenser.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
