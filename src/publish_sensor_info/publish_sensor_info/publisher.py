import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from publish_sensor_info.msg import Greeting

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(Greeting, 'greeting_topic', 10)
        timer_period = 2  # Sekunden
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Greeting()
        if self.i % 2 == 0:
            msg.message = 'Hello'
        else:
            msg.message = 'Bonjour'
        msg.id = self.i
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: ID = {msg.id}, Message = {msg.message}')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()