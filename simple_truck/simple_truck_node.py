import rclpy
from rclpy.node import Node
from simple_truck.truck_model import Truck
from simple_truck.command_processor import SimpleCommandProcessor
from std_msgs.msg import String


class SimpleTruckNode(Node):

    def __init__(self):
        super().__init__('simple_truck_node')
        
        # ROS communication properties
        self.state_publisher = self.create_publisher(String, 'simple_truck/status', 10)
        self.speed_publisher = self.create_publisher(String, 'simple_truck/speed', 10)
        self.subscription = self.create_subscription(String, 'simple_truck/commands', self.listener_callback, 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.truck = Truck()
        self.command_processor = SimpleCommandProcessor()

    def timer_callback(self):
        self.truck.drive()
        self.publish_state()
        self.publish_speed()

    def publish_speed(self):
        msg = String()
        msg.data = str(self.truck.get_speed())
        self.speed_publisher.publish(msg)

    def publish_state(self):
        msg = String()
        msg.data = self.truck.get_status()
        self.state_publisher.publish(msg)
        self.get_logger().info(f'{msg.data}')

    def listener_callback(self, msg):
        self.get_logger().info(f'received {msg.data}')
        self.command_processor.process_string_command(msg.data, self.truck)
        

def main(args=None):
    rclpy.init(args=args)

    truck_node = SimpleTruckNode()

    rclpy.spin(truck_node)

    truck_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()