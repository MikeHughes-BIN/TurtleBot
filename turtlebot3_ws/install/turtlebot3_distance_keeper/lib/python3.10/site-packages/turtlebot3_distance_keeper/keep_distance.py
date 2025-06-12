import rclpy
from rclpy.node import Node
from stable_baselines3 import PPO
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class DistanceKeeper(Node):
    def __init__(self, model_path):
        super().__init__('distance_keeper')
        # Load the trained model
        print("Befor loading")
        self.model = PPO.load(model_path)
        print("After loading")
        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.create_subscription(String, '/ultrasonic_sensor', self.sensor_callback, 1)
        self.desired_distance = 5.0  # Desired distance in meters
        
        print("Init finished")

    def sensor_callback(self, msg:String):
        # Process sensor data
        current_distance = float(msg.data)
        print(f"Current distance: {current_distance}")
        # Prepare the input to match the model's expected format
        obs = {"obs":[current_distance * 16.6666, self.desired_distance]}
        print(obs)

        # Predict the action (e.g., velocity adjustments)
        action, _ = self.model.predict(obs)
        print("Got the decision from MOdel")
        # Map action to robot movement
        twist = Twist()
        
        print(f"Printing Actions: {action}")
        twist.linear.x = action[0] * 0.22
        print("TwistX", twist.linear.x)
        twist.angular.z = 0.0
        

        # Publish movement command
        self.twist_pub.publish(twist)

def main():
    rclpy.init()
    model_path = "/home/turtle1/turtlebot3_ws/src/turtlebot3/turtlebot3_distance_keeper/turtlebot3.zip"
    keeper = DistanceKeeper(model_path)
    rclpy.spin(keeper)
    keeper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
