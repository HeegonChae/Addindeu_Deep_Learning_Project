import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time
import threading

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.cap = cv2.VideoCapture('./data/umin2.MOV')
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)

        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.publisher_ = self.create_publisher(Image, 'video_frames', qos_profile)
        self.string_publisher_ = self.create_publisher(String, 'video_control', qos_profile) 

        self.timer = self.create_timer(1.0 / self.fps, self.publish_video)
        
        self.bridge = CvBridge()
        start_message = String()
        start_message.data = f"START {self.frame_width} {self.frame_height} {self.fps}"
        self.string_publisher_.publish(start_message) 
        self.get_logger().info(f"Published: {start_message.data}")
        time.sleep(1) 
        self.running = True

        self.thread = threading.Thread(target=self.check_for_quit)
        self.thread.start()

    def publish_video(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published frame")
        else:
            end_message = String()
            end_message.data = 'END'
            self.string_publisher_.publish(end_message) 
            self.get_logger().info(f"Published: {end_message.data}")
            rclpy.shutdown()

    def check_for_quit(self):
        while self.running:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                rclpy.shutdown()
                sys.exit()

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    rclpy.spin(video_publisher)
    video_publisher.destroy_node()
    rclpy.shutdown()
    sys.exit()

if __name__ == '__main__':
    main()
