import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, sys

class FakeCamera(Node):
    def __init__(self, path):
        super().__init__('fake_camera')
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        img = cv2.imread(path)
        img = cv2.resize(img, (640, 480))
        self.msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.create_timer(0.033, self.publish)
        self.get_logger().info(f'Publishing {path} at 30fps')

    def publish(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)

def main():
    rclpy.init()
    rclpy.spin(FakeCamera(sys.argv[1]))
    rclpy.shutdown()

if __name__ == '__main__':
    main()
