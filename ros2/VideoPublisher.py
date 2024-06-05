import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
import time

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.publisher_ = self.create_publisher(Image, 'video_topic', qos_profile)
        self.signal_publisher_ = self.create_publisher(String, 'video_signal', qos_profile)
        self.timer = self.create_timer(0.1, self.publish_frame)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture('./data/umin2.MOV')
        self.video_started = False
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)

    def publish_frame(self):
        if not self.video_started:
            start_signal = f"START {self.frame_width} {self.frame_height} {self.fps}"
            self.signal_publisher_.publish(String(data=start_signal))
            self.video_started = True
            print("Send Start Signal")
        
        time.sleep(1.0)

        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
        else:
            self.signal_publisher_.publish(String(data="END"))
            print("Send End Signal")
            self.destroy_node()
            rclpy.shutdown()

    def run(self):
        rclpy.spin(self)
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()

    def check_for_exit():
        if cv2.waitKey(1) & 0xFF == ord('q'):
            video_publisher.signal_publisher_.publish(String(data="END"))
            video_publisher.destroy_node()
            rclpy.shutdown()

    while rclpy.ok():
        rclpy.spin_once(video_publisher, timeout_sec=0.1)
        check_for_exit()

if __name__ == '__main__':
    main()
