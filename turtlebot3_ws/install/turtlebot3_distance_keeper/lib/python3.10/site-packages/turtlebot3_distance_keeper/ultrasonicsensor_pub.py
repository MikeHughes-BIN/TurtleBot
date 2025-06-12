import rclpy 
from rclpy.node import Node  
from std_msgs.msg import String 
from gpiozero import DistanceSensor  

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('ultrasonic_sensor')  
        self.sensor = DistanceSensor(9, 10)        
        timer_period = 0.1 #Set the timer period for sensor data publishing (in seconds)

        self.sonar_publisher = self.create_publisher(String, 'ultrasonic_sensor', 1)
        self.sonar_timer = self.create_timer(timer_period, self.publish_sonar)
        
    def publish_sonar(self):
        msg = String()  #Create a new String message
        msg.data = "%s" % self.sensor.distance  #Message distance mesuared by sensor
        self.sonar_publisher.publish(msg)  #Publish the message
        self.get_logger().info('Publishing: "%s"' % msg.data)  #Log the published data
        
    def __del__(self):
        self.sensor.close()  #Close the distance sensor

def main():
    rclpy.init()  
    sensorPublisher = SensorPublisher() 

    try:
        rclpy.spin(sensorPublisher) 
    except KeyboardInterrupt:
        print("Interrupted by keyboard")
    finally:
        sensorPublisher.sensor.close()
        sensorPublisher.destroy_node()
        rclpy.shutdown() 

if __name__ == '__main__':
    main() 