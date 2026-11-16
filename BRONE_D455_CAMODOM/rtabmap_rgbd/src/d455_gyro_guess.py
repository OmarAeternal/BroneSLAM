#!/usr/bin/env python3
"""
Tebakan rotasi dari gyro D455 untuk odometri kamera (rgbd_odometry)
=============================================================================
Gyro yang dipakai adalah IMU di DALAM kamera D455 (bukan sensor robot), jadi
odometri tetap "kamera saja": RGB + depth + gyro D455, tanpa encoder roda.

Cara kerja (pola guess_frame_id RTAB-Map):
  node ini     : TF  odom_gyro -> base_link   (yaw = integral gyro, translasi 0)
  rgbd_odometry: TF  odom      -> odom_gyro   (koreksi visual)
  hasil        : odom -> base_link = pose odometri kamera (seperti biasa)

Antara dua frame kamera, rgbd_odometry memakai perubahan TF odom_gyro->base_link
sebagai tebakan awal registrasi. Akibatnya:
  * saat berputar di tempat (scene minim tekstur), rotasi tetap terlacak;
  * saat VO gagal lalu reset, odometri melanjutkan dari pose terakhir + rotasi gyro;
  * bias/drift gyro tidak menumpuk selama VO berhasil (dikoreksi di odom->odom_gyro).

Sumbu: kecepatan sudut diputar dari frame IMU (header.frame_id) ke base_link lewat
TF statis, jadi juga benar untuk D455 asli (frame IMU optik: yaw = -gyro_y).
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
import tf2_ros


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


class GyroGuessNode(Node):
    def __init__(self):
        super().__init__('d455_gyro_guess')
        self.declare_parameter('guess_frame_id', 'odom_gyro')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('bias_calib_sec', 2.0)   # robot diam di awal -> estimasi bias
        self.guess_frame = self.get_parameter('guess_frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.calib_sec = float(self.get_parameter('bias_calib_sec').value)

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
        self.tf_pub = tf2_ros.TransformBroadcaster(self)

        self.R_row = None          # baris ke-3 rotasi imu->base (komponen z / yaw)
        self.yaw = 0.0
        self.last_t = None
        self.last_wz = 0.0
        self.bias = 0.0
        self.calib = []            # (t, wz) selama kalibrasi
        self.t0 = None
        self.calibrated = self.calib_sec <= 0.0

        self.create_subscription(Imu, '/imu/data', self.imu_cb, 50)
        self.get_logger().info(f'Gyro guess D455 aktif: TF {self.guess_frame} -> {self.base_frame} '
                               f'(kalibrasi bias {self.calib_sec:.1f} s, robot harus diam)')

    def _yaw_rate(self, msg):
        if self.R_row is None:
            try:
                tf = self.tf_buf.lookup_transform(self.base_frame, msg.header.frame_id, Time())
            except Exception:
                return None
            self.R_row = quat_to_rot(tf.transform.rotation)[2]
            self.get_logger().info(f'Rotasi {msg.header.frame_id}->{self.base_frame}: '
                                   f'wz_base = {self.R_row[0]:+.2f}*gx {self.R_row[1]:+.2f}*gy {self.R_row[2]:+.2f}*gz')
        w = msg.angular_velocity
        return self.R_row[0] * w.x + self.R_row[1] * w.y + self.R_row[2] * w.z

    def imu_cb(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        wz = self._yaw_rate(msg)
        if wz is None:
            return
        if self.t0 is None:
            self.t0 = t

        if not self.calibrated:
            self.calib.append(wz)
            if t - self.t0 >= self.calib_sec:
                b = sum(self.calib) / len(self.calib)
                self.calibrated = True
                if abs(b) > 0.02:   # bias D455 asli ~0.001 rad/s -> robot ternyata bergerak
                    self.get_logger().warn(f'Kalibrasi ditolak (rata-rata {b:+.4f} rad/s, robot bergerak?). '
                                           'Bias = 0; VO tetap mengoreksi drift.')
                else:
                    self.bias = b
                    self.get_logger().info(f'Bias gyro yaw = {self.bias:+.5f} rad/s ({len(self.calib)} sampel)')
            wz_c = 0.0
        else:
            wz_c = wz - self.bias

        if self.last_t is not None:
            dt = t - self.last_t
            if 0.0 < dt < 0.5:
                # trapesium: IMU di sim di-hold antar sampel, di D455 asli 200 Hz
                self.yaw = math.atan2(math.sin(self.yaw + 0.5 * (wz_c + self.last_wz) * dt),
                                      math.cos(self.yaw + 0.5 * (wz_c + self.last_wz) * dt))
            elif dt <= 0.0:
                return
        self.last_t = t
        self.last_wz = wz_c

        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self.guess_frame
        tf.child_frame_id = self.base_frame
        tf.transform.rotation.z = math.sin(self.yaw * 0.5)
        tf.transform.rotation.w = math.cos(self.yaw * 0.5)
        self.tf_pub.sendTransform(tf)


def main():
    rclpy.init()
    node = GyroGuessNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
