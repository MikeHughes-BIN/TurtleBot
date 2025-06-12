import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import time

class OdomResetter(Node):
    def __init__(self):
        super().__init__('reset_odom')

        # Set up the odometry reset publisher
        self.reset_odom_pub = self.create_publisher(Empty, '/odom', 10)

        # Reset odometry (messages take a few iterations to get through)
        self.reset_odometry()

    def reset_odometry(self):
        timer = time.time()
        while time.time() - timer < 0.25:
            self.reset_odom_pub.publish(Empty())
            self.get_logger().info("Publishing odometry reset message...")
            time.sleep(0.05)  # Small delay to allow multiple messages to be processed

def main():
    rclpy.init()
    node = OdomResetter()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
