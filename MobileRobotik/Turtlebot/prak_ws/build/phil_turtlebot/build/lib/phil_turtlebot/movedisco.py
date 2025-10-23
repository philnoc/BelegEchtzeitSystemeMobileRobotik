#!/usr/bin/env python3
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from irobot_create_msgs.msg import HazardDetectionVector, AudioNote, AudioNoteVector
from irobot_create_msgs.action import Undock, Dock
from builtin_interfaces.msg import Duration

from cv_bridge import CvBridge
import cv2


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class Phase(Enum):
    UNDOCK = 0
    DRIVE_TILL_HAZARD = 1
    BACKUP_2S = 2
    SNAPSHOT = 3
    ROTATE_180 = 4
    DRIVE_TIMED = 5
    DOCK = 6
    DONE = 7


class Routine(Node):
    def __init__(self):
        super().__init__('tb4_auto_routine')

        # ------- Parameter -------
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('hazard_topic', '/hazard_detection')
        self.declare_parameter('camera_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('image_path', 'detectedObj.png')

        self.declare_parameter('forward_speed', 0.15)       # m/s
        self.declare_parameter('reverse_speed', -0.12)      # m/s
        self.declare_parameter('angular_speed_max', 0.6)    # rad/s
        self.declare_parameter('angular_speed_min', 0.15)   # rad/s
        self.declare_parameter('angle_tol_deg', 3.0)        # deg

        # nach OBJ: etwas weiter zurück → 2.5s
        self.declare_parameter('backup_time', 2.5)          # s backward after hazard (war 2.0)
        self.declare_parameter('max_forward_time', 30.0)    # s safety cap while searching
        # Rückweg ca. 1s kürzer als Hinweg
        self.declare_parameter('return_time_offset_s', 1.0) # s shorter on the way back

        self.declare_parameter('undock_action', '/undock')
        self.declare_parameter('dock_action', '/dock')
        self.declare_parameter('audio_topic', '/cmd_audio')

        # ------- Pub/Sub/Actions -------
        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)

        self.audio_pub = self.create_publisher(
            AudioNoteVector, self.get_parameter('audio_topic').value, 10)

        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self.odom_cb, qos_profile_sensor_data)

        self.hazard_sub = self.create_subscription(
            HazardDetectionVector, self.get_parameter('hazard_topic').value,
            self.hazard_cb, qos_profile_sensor_data)

        self.image_sub = None  # wird nur in Phase SNAPSHOT angelegt
        self.bridge = CvBridge()

        self.undock_client = ActionClient(self, Undock, self.get_parameter('undock_action').value)
        self.dock_client   = ActionClient(self, Dock,   self.get_parameter('dock_action').value)

        # ------- State -------
        self.phase = Phase.UNDOCK
        self.phase_started = self.get_clock().now()
        self.forward_started = None
        self.forward_elapsed = 0.0  # gemessene Vorwärts-Zeit bis Hazard
        self.return_drive_time = 0.0

        self.current_yaw = None
        self.target_yaw = None

        # Timer-Loop (10 Hz)
        self.timer = self.create_timer(0.1, self.step)

        self.get_logger().info('Starte Routine…')
        self.start_undock()

    # ========= Callbacks =========
    def odom_cb(self, msg: Odometry):
        self.current_yaw = yaw_from_quat(msg.pose.pose.orientation)

    def hazard_cb(self, msg: HazardDetectionVector):
        if self.phase == Phase.DRIVE_TILL_HAZARD and msg.detections:
            # Fahrzeit bis Hazard stoppen
            now = self.get_clock().now()
            self.forward_elapsed = (now - self.forward_started).nanoseconds * 1e-9 if self.forward_started else 0.0
            self.get_logger().info(f'Hazard erkannt – gefahrene Zeit: {self.forward_elapsed:.2f} s')
            self.switch(Phase.BACKUP_2S)

    def image_cb(self, msg: Image):
        # Nur ein Frame speichern, dann Abo beenden
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            path = self.get_parameter('image_path').value
            cv2.imwrite(path, cv_img)
            self.get_logger().info(f'Bild gespeichert: {path}')
        except Exception as e:
            self.get_logger().error(f'Bild speichern fehlgeschlagen: {e}')
        finally:
            if self.image_sub:
                self.destroy_subscription(self.image_sub)
                self.image_sub = None
            self.switch(Phase.ROTATE_180)

    # ========= Helpers =========
    def stop(self):
        self.cmd_pub.publish(Twist())

    def elapsed_phase(self):
        return (self.get_clock().now() - self.phase_started).nanoseconds * 1e-9

    def switch(self, new_phase: Phase):
        self.stop()
        self.phase = new_phase
        self.phase_started = self.get_clock().now()
        self.get_logger().info(f'→ Phase: {self.phase.name}')

        if new_phase == Phase.DRIVE_TILL_HAZARD:
            # Musik: Indiana Jones starten, während wir suchen
            self.play_indiana_jones()
            self.forward_started = self.get_clock().now()

        elif new_phase == Phase.BACKUP_2S:
            # Suche-Musik beim Rücksetzen stoppen
            self.stop_audio()

        elif new_phase == Phase.SNAPSHOT:
            # Einmaliges Bild abonnieren
            topic = self.get_parameter('camera_topic').value
            self.image_sub = self.create_subscription(
                Image, topic, self.image_cb, qos_profile_sensor_data)
            # Falls in 3 s kein Bild kommt: weiter
            self.create_timer(3.0, self._snapshot_timeout)

        elif new_phase == Phase.ROTATE_180:
            # Zielyaw setzen, sobald Odom da ist (notfalls in step() nachziehen)
            if self.current_yaw is not None:
                self.target_yaw = self._wrap(self.current_yaw + math.pi)
            else:
                self.target_yaw = None

        elif new_phase == Phase.DRIVE_TIMED:
            # Rückweg: etwas kürzer als Hinweg (≈1s)
            offset = float(self.get_parameter('return_time_offset_s').value)
            self.return_drive_time = max(0.0, self.forward_elapsed - offset)
            self.get_logger().info(f'Rückweg-Fahrzeit: {self.return_drive_time:.2f} s')

        elif new_phase == Phase.DOCK:
            # vor Docken Audio sicher aus
            self.stop_audio()
            self.start_dock()

    def _snapshot_timeout(self):
        if self.phase == Phase.SNAPSHOT and self.image_sub is not None:
            self.get_logger().warn('Kein Kamerabild empfangen – überspringe.')
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
            self.switch(Phase.ROTATE_180)

    @staticmethod
    def _wrap(a):
        return math.atan2(math.sin(a), math.cos(a))

    # ========= Audio =========
    def publish_notes(self, notes_hz_and_sec, append=False):
        vec = AudioNoteVector()
        vec.append = bool(append)
        for f_hz, secs in notes_hz_and_sec:
            n = AudioNote()
            n.frequency = int(f_hz)   # int muss es sein
            s = float(secs)
            sec = int(s)
            nsec = int((s - sec) * 1e9)
            n.max_runtime = Duration(sec=sec, nanosec=nsec)
            vec.notes.append(n)
        self.audio_pub.publish(vec)

    def stop_audio(self):
        vec = AudioNoteVector()
        vec.append = False
        vec.notes = []  # leere Liste leert die Queue
        self.audio_pub.publish(vec)

    def play_indiana_jones(self):
        # Indiana Jones – deine Arrays, ms-Dauern:
        Jonesnotes=[330,349,392,523,294,330,349,392,440,494,698,440,494,523,587,659,
                    330,349,392,523,587,659,698,392,392,659,587,392,659,587,392,659,
                    587,392,698,659,587,523]
        Jonesdurations=[300,100,400,800,300,100,1200,300,100,400,800,300,100,400,400,400,
                        300,100,400,800,300,100,1200,300,100,400,300,100,400,300,100,400,
                        300,100,400,300,100,1200]
        seq=[(f, ms/1000.0) for f,ms in zip(Jonesnotes, Jonesdurations)]
        self.publish_notes(seq, append=False)

    # ========= Actions =========
    def start_undock(self):
        self.get_logger().info('Warte auf /undock…')
        self.undock_client.wait_for_server()
        fut = self.undock_client.send_goal_async(Undock.Goal())
        fut.add_done_callback(self._undock_sent)

    def _undock_sent(self, fut):
        h = fut.result()
        if not h or not h.accepted:
            self.get_logger().error('Undock abgelehnt.')
            self.switch(Phase.DONE)
            return
        self.get_logger().info('Undock akzeptiert – warte auf Result…')
        h.get_result_async().add_done_callback(self._undock_result)

    def _undock_result(self, fut):
        try:
            res = fut.result().result
            self.get_logger().info(f'Undock beendet. is_docked={getattr(res,"is_docked",None)}')
        except Exception as e:
            self.get_logger().warn(f'Undock Result: {e}')
        # KEIN Super Mario mehr – direkt Suche starten
        self.switch(Phase.DRIVE_TILL_HAZARD)

    def start_dock(self):
        self.get_logger().info('Warte auf /dock…')
        self.dock_client.wait_for_server()
        fut = self.dock_client.send_goal_async(Dock.Goal())
        fut.add_done_callback(self._dock_sent)

    def _dock_sent(self, fut):
        h = fut.result()
        if not h or not h.accepted:
            self.get_logger().error('Dock abgelehnt.')
            self.switch(Phase.DONE)
            return
        self.get_logger().info('Dock akzeptiert – warte auf Result…')
        h.get_result_async().add_done_callback(self._dock_result)

    def _dock_result(self, fut):
        try:
            res = fut.result().result
            self.get_logger().info(f'Dock beendet. is_docked={getattr(res,"is_docked",None)}')
        except Exception as e:
            self.get_logger().warn(f'Dock Result: {e}')
        self.switch(Phase.DONE)

    # ========= Main loop =========
    def step(self):
        v_fwd = float(self.get_parameter('forward_speed').value)
        v_rev = float(self.get_parameter('reverse_speed').value)
        ang_max = float(self.get_parameter('angular_speed_max').value)
        ang_min = float(self.get_parameter('angular_speed_min').value)
        tol = math.radians(float(self.get_parameter('angle_tol_deg').value))

        if self.phase == Phase.DRIVE_TILL_HAZARD:
            # failsafe: max_forward_time
            if self.elapsed_phase() > float(self.get_parameter('max_forward_time').value):
                self.forward_elapsed = self.elapsed_phase()
                self.get_logger().warn(f'Keine Hazard-Detection in {self.forward_elapsed:.1f}s – mache weiter.')
                self.switch(Phase.BACKUP_2S)
            else:
                cmd = Twist(); cmd.linear.x = v_fwd
                self.cmd_pub.publish(cmd)

        elif self.phase == Phase.BACKUP_2S:
            if self.elapsed_phase() <= float(self.get_parameter('backup_time').value):
                cmd = Twist(); cmd.linear.x = v_rev
                self.cmd_pub.publish(cmd)
            else:
                self.switch(Phase.SNAPSHOT)

        elif self.phase == Phase.ROTATE_180:
            if self.target_yaw is None and self.current_yaw is not None:
                self.target_yaw = self._wrap(self.current_yaw + math.pi)
                self.get_logger().info('Yaw-Target (180°) gesetzt.')
            if self.current_yaw is None or self.target_yaw is None:
                return
            err = self._wrap(self.target_yaw - self.current_yaw)
            if abs(err) > tol:
                omega = max(-ang_max, min(ang_max, 1.5 * err))
                if abs(omega) < ang_min:
                    omega = ang_min if omega >= 0 else -ang_min
                cmd = Twist(); cmd.angular.z = omega
                self.cmd_pub.publish(cmd)
            else:
                self.switch(Phase.DRIVE_TIMED)

        elif self.phase == Phase.DRIVE_TIMED:
            if self.elapsed_phase() <= max(0.0, self.return_drive_time):
                cmd = Twist(); cmd.linear.x = v_fwd
                self.cmd_pub.publish(cmd)
            else:
                self.switch(Phase.DOCK)

        elif self.phase in (Phase.DOCK, Phase.DONE):
            if self.phase == Phase.DONE and rclpy.ok():
                self.stop()
                self.get_logger().info('Routine abgeschlossen. Beende Node.')
                rclpy.shutdown()


def main():
    rclpy.init()
    node = Routine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
