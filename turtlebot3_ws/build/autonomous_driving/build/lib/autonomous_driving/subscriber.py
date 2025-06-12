import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32


class DistanceSubscriber(Node):
    def __init__(self):
        super().__init__("distance_subscriber")
        self.subscription = self.create_subscription(
            Float32, "distance", self.listener_callback, 1
        )
        self.subscription  # prevent unused variable warning
        self.twist = Twist()
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 1)

    def listener_callback(self, msg):
        # immer wenn nachricht kommt mache:
        self.get_logger().info('I heard: "%s"' % msg.data)
        #if too close, slow down
        if msg.data > 0.3:
            self.twist.linear.x += 0.01
        elif msg.data < 0.3 and msg.data > 0.2:
            self.twist.linear.x = 0.0
        else: 
            self.twist.linear.x = -0.05
        self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)

    distance_subscriber = DistanceSubscriber()

    rclpy.spin(distance_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    distance_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
