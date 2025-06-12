import rclpy
from rclpy.node import Node
from publish_sensor_info.msg import Greeting  # Replace with your actual package name if different

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            Greeting,
            'greeting_topic',
            self.listener_callback,
            10
        )
        self.subscription  # Verhindert ungenutzte Variable Warnung

    def listener_callback(self, msg):
        self.get_logger().info('Received: ID = %d, Message = %s' % (msg.id, msg.message))
        with open('received_messages.txt', 'a') as f:
            f.write(f'ID = {msg.id}, Message = {msg.message}\n')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()