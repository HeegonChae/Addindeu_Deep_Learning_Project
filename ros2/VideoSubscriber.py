import cv2
import os
from sensor_msgs.msg import Image
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from datetime import datetime

class VideoSubscriber(Node):
    def __init__(self):
        super().__init__('video_subscriber')
        self.subscription = self.create_subscription(Image, 'video_topic', self.callback, 10)
        self.signal_subscription = self.create_subscription(String, 'video_signal', self.signal_callback, 10)
        self.bridge = CvBridge()
        self.video_writer = None
        self.folder_path = None
        self.recording = False

    def callback(self, msg):
        if not self.recording:
            return
        
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if self.video_writer is not None:
            self.video_writer.write(frame)
        cv2.imshow("Video", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()
            rclpy.shutdown()

    def signal_callback(self, msg):
        data = msg.data.split()
        if data[0] == "START":
            self.get_logger().info("Received Start Signal")
            width = int(data[1])
            height = int(data[2])
            fps = float(data[3])
            now = datetime.now().strftime("%y%m%d_%H:%M:%S")
            self.folder_path = f"./video/{now}"
            os.makedirs(self.folder_path, exist_ok=True)
            video_path = os.path.join(self.folder_path, "output.mp4")
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(video_path, fourcc, fps, (width, height))
            self.recording = True
        elif data[0] == "END":
            self.get_logger().info("Received End Signal")
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None
            self.recording = False
            cv2.destroyAllWindows()

    def run(self):
        rclpy.spin(self)
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    video_subscriber = VideoSubscriber()

    def check_for_exit():
        if cv2.waitKey(1) & 0xFF == ord('q'):
            video_subscriber.destroy_node()
            rclpy.shutdown()

    while rclpy.ok():
        rclpy.spin_once(video_subscriber, timeout_sec=0.1)
        check_for_exit()

if __name__ == '__main__':
    main()
