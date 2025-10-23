#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
from builtin_interfaces.msg import Duration
import time


class MusicTest(Node):
    def __init__(self):
        super().__init__('music_test')
        self.pub = self.create_publisher(AudioNoteVector, '/cmd_audio', 10)
        self.get_logger().info("Starte Musik-Test: Super Mario + Indiana Jones")

        # Erst Mario, dann Jones
        self.play_mario()
        self.get_logger().info("Mario-Theme gespielt. Warte 2 Sekunden…")
        time.sleep(2.0)
        self.play_indiana_jones()
        self.get_logger().info("Indiana-Jones-Theme gespielt. Test fertig.")

    # ---------- Hilfsfunktion ----------
    def publish_notes(self, seq):
        msg = AudioNoteVector()
        msg.append = False
        for freq, dur in seq:
            n = AudioNote()
            n.frequency = int(freq)
            s = float(dur)
            sec = int(s)
            nsec = int((s - sec) * 1e9)
            n.max_runtime = Duration(sec=sec, nanosec=nsec)
            msg.notes.append(n)
        self.pub.publish(msg)
        # kleine Pause, damit sich das abspielen kann
        time.sleep(sum(d for _, d in seq) + 0.5)

    # ---------- Mario ----------
    def play_mario(self):
        NOTE_E7=2637; NOTE_G7=3136; NOTE_A7=3520; NOTE_B7=3951
        NOTE_E6=1319; NOTE_G6=1568; NOTE_A6=1760; NOTE_C7=2093
        REST=0
        Mariomelody=[NOTE_E7, NOTE_E7, REST, NOTE_E7, REST, NOTE_C7, NOTE_E7,
                     REST, NOTE_G7, REST, REST, REST, NOTE_G6, REST, REST, REST]
        MarionoteDurations=[12,12,12,12,12,12,12,12,9,12,12,12,12,12,12,12]
        seq=[(f, 1.0/d) for f,d in zip(Mariomelody, MarionoteDurations)]
        self.get_logger().info("▶️ Spiele Super Mario Theme")
        self.publish_notes(seq)

    # ---------- Indiana Jones ----------
    def play_indiana_jones(self):
        Jonesnotes=[330,349,392,523,294,330,349,392,440,494,698,440,494,523,587,659,
                    330,349,392,523,587,659,698,392,392,659,587,392,659,587,392,659,
                    587,392,698,659,587,523]
        Jonesdurations=[300,100,400,800,300,100,1200,300,100,400,800,300,100,400,400,400,
                        300,100,400,800,300,100,1200,300,100,400,300,100,400,300,100,400,
                        300,100,400,300,100,1200]
        seq=[(f, ms/1000.0) for f,ms in zip(Jonesnotes,Jonesdurations)]
        self.get_logger().info("▶️ Spiele Indiana Jones Theme")
        self.publish_notes(seq)


def main():
    rclpy.init()
    node = MusicTest()
    # keine spin nötig, läuft synchron durch
    rclpy.shutdown()


if __name__ == '__main__':
    main()
