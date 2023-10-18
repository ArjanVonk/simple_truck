import rclpy
from rclpy.node import Node
from simple_truck.truck_model import Truck
from std_msgs.msg import String


class SimpleTruckNode(Node):

    def __init__(self):
        super().__init__('simple_truck_node')
        self.publisher_ = self.create_publisher(String, 'simple_truck_status', 10)
        self.subscription = self.create_subscription(String, 'simple_truck_listener', self.listener_callback, 10)

        self.truck = Truck()
        self.truck.start_truck()
        self.truck.set_throttle_percentage(50)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        self.truck.drive()
        self.publish_info()

    def publish_info(self):
        msg = String()
        msg.data = self.truck.get_status()
        self.publisher_.publish(msg)
        self.get_logger().info(f'{msg.data}')

    def listener_callback(self, msg):
        message = msg.data.split("_",1)
        self.get_logger().info(f'received {message}')
        match message:
            case ["start"]:
                self.truck.start_truck()
            case ["stop"]: 
                self.truck.turn_off_truck()
            case ["throttle", *throttle_val]:
                self.get_logger().info(f'{throttle_val}')
                self.truck.set_throttle_percentage(float(throttle_val[0]))
        

def main(args=None):
    rclpy.init(args=args)

    truck_node = SimpleTruckNode()

    rclpy.spin(truck_node)

    truck_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()