import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Range


class UltraDisp(Node):
    def __init__(self):
        super().__init__('ultra_disp')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.disp_pub = self.create_publisher(String, '/display', 10)

        self.sub_range = self.create_subscription(
            Range,
            '/ultrasonic_range',
            self.range_callback,
            10
        )

        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.move)

        self.last_range = None

    def range_callback(self, msg: Range):
        self.last_range = msg.range
        disp_msg = String()
        disp_msg.data = f"Entfernung: {msg.range:.2f} cm"
        self.disp_pub.publish(disp_msg)

    def move(self):
        msg = Twist()
        if self.last_range is None:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        else:
            if self.last_range < 0.3:
                msg.linear.x = 0.0
                msg.angular.z = 0.5
            else:
                msg.linear.x = 0.3
                msg.angular.z = 0.0

        self.cmd_pub.publish(msg)

    def stop(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_pub.publish(stop_msg)
        self.get_logger().info("Stop command sent on shutdown.")


def main(args=None):
    rclpy.init(args=args)
    node = UltraDisp()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # hier sauber stoppen
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

