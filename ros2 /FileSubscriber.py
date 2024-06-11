import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import os
from cv_bridge import CvBridge
from datetime import datetime

class FileSubscriber(Node):
    def __init__(self):
        super().__init__('file_subscriber')
        
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.image_subscription = self.create_subscription(
            Image,
            'image_files',
            self.image_listener_callback,
            qos_profile)
        
        self.start_subscription = self.create_subscription(
            String,
            'start_signal',
            self.start_signal_callback,
            qos_profile)
        
        self.end_subscription = self.create_subscription(
            String,
            'end_signal',
            self.end_signal_callback,
            qos_profile)

        self.bridge = CvBridge()
        self.file_path = './received_files/detection'
        self.current_folder = None

        self.get_logger().info('FileSubscriber node has been initialized')

    def image_listener_callback(self, msg):
        if self.current_folder is None:
            self.get_logger().warning('Start signal not received yet. Ignoring image.')
            return

        self.get_logger().info('Received an image file')
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        filename = msg.header.frame_id
        image_path = os.path.join(self.current_folder, filename)
        cv2.imwrite(image_path, frame)
        self.get_logger().info(f'Image saved to {image_path}')

    def start_signal_callback(self, msg):
        if msg.data == "start":
            self.current_folder = os.path.join(self.file_path, datetime.now().strftime("%y%m%d_%H:%M:%S"))
            if not os.path.exists(self.current_folder):
                os.makedirs(self.current_folder)
            self.get_logger().info(f'Received start signal, created folder: {self.current_folder}')

    def end_signal_callback(self, msg):
        if msg.data == "end":
            self.get_logger().info('Received end signal, waiting for other publishers...')
            self.current_folder = None

def main(args=None):
    rclpy.init(args=args)
    file_subscriber = FileSubscriber()
    rclpy.spin(file_subscriber)
    file_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
