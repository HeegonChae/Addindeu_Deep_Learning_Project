import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import time
import os

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')

        base_path = './data/lstm'
        subdirs = sorted([d for d in os.listdir(base_path) if os.path.isdir(os.path.join(base_path, d))])
        if not subdirs:
            self.get_logger().error("No subdirectories found in the base path.")
            rclpy.shutdown()
            sys.exit()

        latest_subdir = os.path.join(base_path, subdirs[-1])
        files = sorted(os.listdir(latest_subdir))
        video_files = [f for f in files if f.endswith('.mp4') or f.endswith('.MOV')]

        if not video_files:
            self.get_logger().error("No video files found in the latest subdirectory.")
            rclpy.shutdown()
            sys.exit()

        video_file_path = os.path.join(latest_subdir, video_files[0])
        self.cap = cv2.VideoCapture(video_file_path)

        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video file: {video_file_path}")
            rclpy.shutdown()
            sys.exit()

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

        self.bridge = CvBridge()
        start_message = String()
        start_message.data = f"START {self.frame_width} {self.frame_height} {self.fps}"
        self.string_publisher_.publish(start_message)
        self.get_logger().info(f"Published: {start_message.data}")
        time.sleep(1)

        self.publish_video()

    def publish_video(self):
        while True:
            ret, frame = self.cap.read()
            if ret:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.publisher_.publish(msg)
                self.get_logger().info("Published frame ")
            else:
                time.sleep(1)
                end_message = String()
                end_message.data = 'END'
                self.string_publisher_.publish(end_message)
                self.get_logger().info("Published: END")
                self.cap.release()
                break

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()
    video_publisher.destroy_node()
    rclpy.shutdown()
    sys.exit()

if __name__ == '__main__':
    main()
