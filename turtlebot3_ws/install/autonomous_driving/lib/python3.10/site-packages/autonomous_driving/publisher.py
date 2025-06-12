import rclpy
from rclpy.node import Node
from gpiozero import DistanceSensor, Button
from std_msgs.msg import String, Bool, Float32


class DistancePublisher(Node):

    def __init__(self):
        super().__init__("distance_publisher")
        timer_period = 0.1  # seconds
        # Sonar
        self.publisher_sonar = self.create_publisher(Float32, "distance", 1)
        self.timer_sonar = self.create_timer(timer_period, self.publishSonar)
        self.sensor = DistanceSensor(9, 10)
        # Bumper
        self.publisher_bumper = self.create_publisher(Bool, "bumper", 1)
        self.timer_bumper = self.create_timer(timer_period, self.publishBumper)
        self.bumper = Button(4, pull_up=False)

    def publishSonar(self):
        msg = Float32()
        msg.data = self.sensor.distance
        self.publisher_sonar.publish(msg)
        self.get_logger().info('Distance: "%s"' % msg.data)

    def publishBumper(self):
        msg = Bool()
        msg.data = self.bumper.is_active
        self.publisher_bumper.publish(msg)
        self.get_logger().info('Bumper: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    distance_publisher = DistancePublisher()
    rclpy.spin(distance_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    distance_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
