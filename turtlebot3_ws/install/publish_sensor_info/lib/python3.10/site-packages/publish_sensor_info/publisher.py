import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from gpiozero import DistanceSensor


class SimplePublisher(Node):

    def __init__(self):
        super().__init__('simple_publisher')
        self.sensor = DistanceSensor(9, 10)
        self.publisher_ = self.create_publisher(Float64, 'gretings', 1)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float64()
        msg.data = self.sensor.distance
        self.publisher_.publish(msg)
        self.get_logger().info('Published: "%.3f"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = SimplePublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()