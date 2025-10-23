import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import String

class CircleDriveNode(Node):
    def __init__(self):
        super().__init__('circle_drive_with_lift_warning')

        # Publisher für Bewegungen
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher für Sound
        self.sound_pub = self.create_publisher(String, '/sound', 10)

        # Subscriber für IMU
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu_values',
            self.imu_callback,
            10
        )

        # Timer für Kreisfahrt
        self.timer = self.create_timer(0.1, self.drive_circle)

        # Letzte Werte
        self.last_z_accel = 9.8
        self.lift_detected = False

        # Schwellenwert: wenn deutlich weniger als 9.8 m/s² → Roboter wird angehoben
        self.lift_threshold = 8.0  # m/s²
        
        # Startup-Sound
        self.startup_timer = self.create_timer(2.0, self.play_startup_sound)
        
        self.warning_timer = None  # erst später anlegen

    def drive_circle(self):
        """Fährt konstant im Kreis"""
        twist = Twist()
        twist.linear.x = 1.2     # vorwärts
        twist.angular.z = 0.5    # Drehen
        self.cmd_pub.publish(twist)

    def imu_callback(self, msg: Imu):
        """Prüft IMU-Daten"""
        z_accel = msg.linear_acceleration.z
        self.last_z_accel = z_accel

        if z_accel < self.lift_threshold and not self.lift_detected:
            self.lift_detected = True
            self.get_logger().warn(f"Lift detected! z_accel={z_accel:.2f}")
            self.start_warning()
        elif z_accel >= self.lift_threshold:
            self.lift_detected = False  # zurücksetzen, wenn wieder normal
            self.stop_warning()

    def play_warning(self):
        """Publiziert einen Warnton"""
        msg = String()
        msg.data = 'notes:440,;durations:1000,'
        self.sound_pub.publish(msg)
        self.get_logger().info("Warning sound sent!")
        
    def start_warning(self):
        if self.warning_timer is None:
            self.warning_timer = self.create_timer(0.5, self.play_warning)
            self.get_logger().info("Warning timer started!")
        
    def stop_warning(self):
        if self.warning_timer is not None:
            self.warning_timer.cancel()
            self.warning_timer = None
            self.get_logger().info("Warning timer stopped!")
        
    def play_startup_sound(self):
        msg = String()
        msg.data = "Supermario"
        self.sound_pub.publish(msg)
        self.get_logger().info("Startup sound SuperMario sent!")

        # Timer nur einmal ausführen
        self.startup_timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = CircleDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Roboter anhalten beim Shutdown
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
        node.get_logger().info("Node stopped, robot halted.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

