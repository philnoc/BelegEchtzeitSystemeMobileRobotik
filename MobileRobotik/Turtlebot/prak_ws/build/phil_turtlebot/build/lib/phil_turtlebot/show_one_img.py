#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ShowOneImage(Node):
    """Abonniert einmal das Kameratopic und zeigt ein Bild mit OpenCV an."""

    def __init__(self):
        super().__init__('show_one_image')
        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image,
            '/oakd/rgb/preview/image_raw',  # ggf. anpassen
            self.callback,
            10)
        self.image_received = False

    def callback(self, msg):
        """Wird beim ersten empfangenen Bild aufgerufen."""
        if self.image_received:
            return
        self.image_received = True
        self.get_logger().info('Bild empfangen, konvertiere...')
        # ROS->OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().info(f'Bildgröße: {frame.shape[1]}x{frame.shape[0]} Pixel')
        # Anzeige
        cv2.imshow('OAK-D Bild', frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        self.get_logger().info('Fenster geschlossen, Node beendet.')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = ShowOneImage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

