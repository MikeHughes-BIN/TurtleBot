import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import cv2

class LidarArcDetector(Node):
    def __init__(self):
        super().__init__('lidar_arc_detector')

        # QoS profile to handle the LiDAR sensor's best-effort communication
        self.qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to the LiDAR /scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            self.qos_profile)

    def lidar_callback(self, msg):
        """
        Callback function to handle incoming LiDAR data.
        """
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Convert polar coordinates to Cartesian coordinates
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        self.visualize_and_save(x, y)

    def visualize_and_save(self, x, y):
        """
        Save the LiDAR scan as an image and detect arcs.
        """
        plt.figure(figsize=(6, 6))
        plt.scatter(x, y, label='Lidar Points', s=10, c='blue')

        plt.axhline(0, color='black', linewidth=0.5)
        plt.axvline(0, color='black', linewidth=0.5)
        plt.grid(False)
        plt.legend()
        plt.xlabel("X (meters)")
        plt.ylabel("Y (meters)")
        plt.title("LiDAR Scan")

        # Convert the plot to an image in memory
        plt.gcf().canvas.draw()
        img = np.frombuffer(plt.gcf().canvas.tostring_rgb(), dtype=np.uint8)
        img = img.reshape(plt.gcf().canvas.get_width_height()[::-1] + (3,))
        plt.close()

        # Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

        # Apply Gaussian Blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Edge detection using Canny
        edges = cv2.Canny(blurred, 50, 150)

        # Detect circles using Hough Circle Transform
        circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, dp=1.2, minDist=2,
                                   param1=80, param2=25, minRadius=5, maxRadius=35)

        # Draw detected circles (arcs can be part of these)
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                center = (i[0], i[1])  # Circle center
                radius = i[2]  # Circle radius

                # Filter out circles that do not match the expected properties of the arc
                if radius < 7:  # Example condition, adjust as needed
                     continue

                # Draw the detected circle
                cv2.circle(img, center, radius, (0, 255, 0), 2)

                # Draw center of circle
                cv2.circle(img, center, 2, (0, 0, 255), 3)

        # Save the image with detected circles
        processed_image_path = "/home/turtle1/Downloads/arc_detected.png"
        cv2.imwrite(processed_image_path, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

        self.get_logger().info(f"Saved scan visualization with detected arcs to {processed_image_path}")

def main(args=None):
    rclpy.init(args=args)
    node = LidarArcDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()