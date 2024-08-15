import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import os
from cv_bridge import CvBridge
import time
import sys

class FilePublisher(Node):
    def __init__(self):
        super().__init__('file_publisher')

        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.image_publisher_ = self.create_publisher(Image, 'image_files', qos_profile)
        self.start_publisher_ = self.create_publisher(String, 'start_signal', qos_profile)
        self.end_publisher_ = self.create_publisher(String, 'end_signal', qos_profile)
        self.bridge = CvBridge()

        self.send_start_signal()
        time.sleep(1)

    def send_initial_files(self):
        base_path = './data/detection'
        subdirs = sorted([d for d in os.listdir(base_path) if os.path.isdir(os.path.join(base_path, d))])
        if not subdirs:
            self.get_logger().error("No subdirectories found in the base path.")
            return
        
        latest_subdir = os.path.join(base_path, subdirs[-1])
        files = sorted(os.listdir(latest_subdir))
        jpg_files = [f for f in files if f.endswith('.jpg')]

        for jpg_file in jpg_files:
            self.send_image_file(latest_subdir, jpg_file)
            time.sleep(1)

        self.send_end_signal()

    def send_image_file(self, path, filename):
        file_path = os.path.join(path, filename)
        image = cv2.imread(file_path)
        if image is not None:
            msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            msg.header.frame_id = filename 
            self.image_publisher_.publish(msg)
            self.get_logger().info(f"Published image file: {filename}")
        else:
            self.get_logger().error(f"Failed to read image file: {filename}")

    def send_start_signal(self):
        msg = String()
        msg.data = 'start'
        self.start_publisher_.publish(msg)
        self.get_logger().info("Published start signal")

    def send_end_signal(self):
        msg = String()
        msg.data = 'end'
        self.end_publisher_.publish(msg)
        self.get_logger().info("Published end signal")

def main(args=None):
    rclpy.init(args=args)
    file_publisher = FilePublisher()
    file_publisher.send_initial_files()
    file_publisher.destroy_node()
    rclpy.shutdown()
    sys.exit()

if __name__ == '__main__':
    main()
