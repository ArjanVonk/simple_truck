import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class SimpleTruckNode(Node):

    def __init__(self):
        super().__init__('simple_truck_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World:'
        self.publisher_.publish(msg)
        self.get_logger().info(f'{msg.data}')


def main(args=None):
    rclpy.init(args=args)

    truck_node = SimpleTruckNode()

    rclpy.spin(truck_node)

    truck_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()