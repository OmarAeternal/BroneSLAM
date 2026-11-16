#!/usr/bin/env python3
"""
Teleop & Reactive D455 Auto-Explorer Node (Holonomic 3-Omniwheel)
Tombol:
  W / S : Maju / Mundur (vx)
  A / D : Geser Kiri / Kanan (vy - Holonomic Strafe)
  Q / E : Putar Kiri / Kanan (wz)
  SPACE : Berhenti (Stop)
  P     : Toggle Reactive Auto-Explore (Menjelajahi ruangan otomatis menggunakan Depth D455)
  X     : Keluar
"""

import sys
import tty
import termios
import select
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class TeleopExplorerNode(Node):
    def __init__(self):
        super().__init__('teleop_explorer')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.auto_mode = False
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        # Jarak objek di sektor Kanan, Tengah, dan Kiri dari FOV 86 derajat D455
        self.min_right = 6.0
        self.min_front = 6.0
        self.min_left = 6.0
        self.turn_dir = 1.0

        self.timer = self.create_timer(0.1, self.timer_callback)

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        # Ganti inf dengan range_max (6.0m) dan abaikan nan
        clean = np.where(np.isinf(ranges), msg.range_max, ranges)
        n = len(clean)
        if n < 9:
            return

        right_sector = clean[:n // 3]
        front_sector = clean[n // 3: 2 * n // 3]
        left_sector = clean[2 * n // 3:]

        vr = right_sector[np.isfinite(right_sector)]
        vf = front_sector[np.isfinite(front_sector)]
        vl = left_sector[np.isfinite(left_sector)]

        self.min_right = float(np.min(vr)) if vr.size > 0 else 6.0
        self.min_front = float(np.min(vf)) if vf.size > 0 else 6.0
        self.min_left = float(np.min(vl)) if vl.size > 0 else 6.0

    def timer_callback(self):
        msg = Twist()
        if self.auto_mode:
            # Navigasi Reaktif berbasis Depth D455: Hindari tembok/perabot & cari ruang terbuka
            if self.min_front < 0.65:
                # Ada tembok/meja di depan -> Berhenti maju dan berputar ke sisi yang lebih lega
                msg.linear.x = 0.0
                if abs(self.min_left - self.min_right) > 0.15:
                    self.turn_dir = 1.0 if self.min_left > self.min_right else -1.0
                msg.angular.z = 0.55 * self.turn_dir
            elif self.min_left < 0.45:
                # Terlalu dekat dinding kiri -> Maju pelan sambil belok/geser ke kanan
                msg.linear.x = 0.22
                msg.linear.y = -0.10
                msg.angular.z = -0.35
            elif self.min_right < 0.45:
                # Terlalu dekat dinding kanan -> Maju pelan sambil belok/geser ke kiri
                msg.linear.x = 0.22
                msg.linear.y = 0.10
                msg.angular.z = 0.35
            else:
                # Ruang depan lega -> Maju mulus sambil mengarah ke sisi yang lebih luas
                msg.linear.x = 0.32
                steer = max(-0.25, min(0.25, 0.15 * (self.min_left - self.min_right)))
                msg.angular.z = steer
        else:
            msg.linear.x = self.vx
            msg.linear.y = self.vy
            msg.angular.z = self.wz

        self.pub.publish(msg)


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = TeleopExplorerNode()

    if not sys.stdin.isatty():
        node.auto_mode = True
        node.get_logger().info('Mode Reactive Auto-Explore Aktif!')
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return

    settings = termios.tcgetattr(sys.stdin)
    print("==========================================================")
    print("  KONTROL ROBOT OMNI + REALSENSE D455 (APARTMENT WORLD)   ")
    print("  W/S: Maju/Mundur | A/D: Geser Kiri/Kanan | Q/E: Putar   ")
    print("  P  : Toggle Reactive Auto-Explore (Bebas Tabrak!)       ")
    print("  SPACE: Stop | X: Keluar                                 ")
    print("==========================================================")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            key = get_key(settings).lower()
            if key == 'w':
                node.auto_mode = False
                node.vx, node.vy, node.wz = 0.38, 0.0, 0.0
            elif key == 's':
                node.auto_mode = False
                node.vx, node.vy, node.wz = -0.35, 0.0, 0.0
            elif key == 'a':
                node.auto_mode = False
                node.vx, node.vy, node.wz = 0.0, 0.32, 0.0
            elif key == 'd':
                node.auto_mode = False
                node.vx, node.vy, node.wz = 0.0, -0.32, 0.0
            elif key == 'q':
                node.auto_mode = False
                node.vx, node.vy, node.wz = 0.0, 0.0, 0.55
            elif key == 'e':
                node.auto_mode = False
                node.vx, node.vy, node.wz = 0.0, 0.0, -0.55
            elif key == ' ':
                node.auto_mode = False
                node.vx, node.vy, node.wz = 0.0, 0.0, 0.0
            elif key == 'p':
                node.auto_mode = not node.auto_mode
                print(f"\r[INFO] Reactive Auto-Explore: {'AKTIF' if node.auto_mode else 'NONAKTIF'}      ")
            elif key == 'x' or key == '\x03':
                break
    finally:
        node.pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
