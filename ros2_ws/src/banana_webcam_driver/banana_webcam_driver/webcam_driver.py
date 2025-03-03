import time
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class WebcamDriver(Node):
    def __init__(self):
        super().__init__('webcam_driver')
        

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT  # Ensure this matches C++
        )

        self.publisher = self.create_publisher(Image, '/webcam/image_raw', qos_profile)

        
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.capture_image)
        self.cap = cv2.VideoCapture(0)

    def capture_image(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture image')
            return

        timestamp = "Time: " + str(time.time())
        cv2.putText(frame, timestamp, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        cv2.imshow('Webcam', frame)
        cv2.waitKey(1)
        self.publisher.publish(msg)
        self.get_logger().info('Published Image')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WebcamDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()