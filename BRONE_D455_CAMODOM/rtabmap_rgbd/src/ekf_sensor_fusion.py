#!/usr/bin/env python3
"""
LAYER 1.5B: Extended Kalman Filter (EKF) Sensor Fusion Node
Referensi: Siegwart et al., Bab 5.6.2 (Extended Kalman Filter, Persamaan 5.44 - 5.51)

Vektor State EKF (6-DoF Planar Holonomic):
  x_t = [x, y, theta, vx, vy, wz]^T

Alur EKF (Sesuai Gambar 5.13 Buku Siegwart):
1. EKF Prediction ("Act"):
   - Memproyeksikan state ke langkah berikutnya dengan fungsi non-linier f(x, dt):
     x_{t|t-1} = x + (vx*cos(theta) - vy*sin(theta)) * dt
     y_{t|t-1} = y + (vx*sin(theta) + vy*cos(theta)) * dt
     theta_{t|t-1} = theta + wz * dt
   - Menghitung Matriks Jacobian F_x = df/dx dan memperbarui kovariansi prediksi:
     Sigma_{t|t-1} = F_x * Sigma_{t-1} * F_x^T + Q_t

2. EKF Measurement Update ("See") dengan Kalman Gain K_t:
   K_t = Sigma_{t|t-1} * H^T * (H * Sigma_{t|t-1} * H^T + R_t)^(-1)
   x_t = x_{t|t-1} + K_t * (z_t - H * x_{t|t-1})
   Sigma_t = (I - K_t * H) * Sigma_{t|t-1}

   Menggabungkan 3 sensor secara asinkron:
   - Update A (/wheel/odom): Mengukur [vx, vy] dengan R_wheel = diag(0.02, 0.02)
   - Update B (/imu/data)  : Mengukur [theta, wz] dengan R_imu = diag(0.0005, 0.001)
   - Update C (/vo/odom)   : Mengukur [vx, vy] dari Kamera RGB-D D455 dengan R_vo DINAMIS!
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros


def euler_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class EkfSensorFusionNode(Node):
    def __init__(self):
        super().__init__('ekf_sensor_fusion')

        # 1. State EKF: x = [x, y, theta, vx, vy, wz]^T
        self.state = np.zeros(6, dtype=np.float64)
        # Matriks Kovariansi State (Sigma_t, 6x6)
        self.Sigma = np.diag([0.001, 0.001, 0.001, 0.01, 0.01, 0.01])

        # Matriks Process Noise (Q_t, 6x6) per detik
        self.Q_rate = np.diag([0.005, 0.005, 0.001, 0.08, 0.08, 0.02])

        self.last_predict_time = None

        # 2. Subscribers (3 Sumber Sensor) & Publishers (/odom + TF odom -> base_link)
        self.create_subscription(Odometry, '/wheel/odom', self.wheel_callback, 20)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 20)
        self.create_subscription(Odometry, '/vo/odom', self.vo_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 20)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info('=============================================================')
        self.get_logger().info('  LAYER 1.5B: EKF SENSOR FUSION NODE AKTIF (Bab 5.6.2)       ')
        self.get_logger().info('  - Melebur: /wheel/odom + /imu/data + /vo/odom -> /odom     ')
        self.get_logger().info('=============================================================')

    def _predict_to_time(self, now_sec: float):
        """
        Langkah 1 EKF: Prediction Step ("Act") dengan Matriks Jacobian F_x.
        """
        if self.last_predict_time is None:
            self.last_predict_time = now_sec
            return

        dt = now_sec - self.last_predict_time
        if dt <= 0.0:
            # Pesan terlambat (mis. /vo/odom dengan stamp citra lama): JANGAN mundurkan
            # waktu prediksi, kalau tidak selang yang sama terintegrasi dua kali
            # (yaw membengkak ~1.5x saat hanya gyro yang memberi info rotasi).
            return
        if dt > 0.5:
            self.last_predict_time = now_sec
            return
        self.last_predict_time = now_sec

        x, y, th, vx, vy, wz = self.state
        cos_t, sin_t = math.cos(th), math.sin(th)

        # Prediksi state non-linier: x_{t|t-1} = f(x_{t-1}, dt)
        self.state[0] = x + (vx * cos_t - vy * sin_t) * dt
        self.state[1] = y + (vx * sin_t + vy * cos_t) * dt
        self.state[2] = normalize_angle(th + wz * dt)

        # Matriks Jacobian F_x = df / dx (6x6) sesuai Persamaan 5.47 Siegwart
        Fx = np.eye(6, dtype=np.float64)
        Fx[0, 2] = (-vx * sin_t - vy * cos_t) * dt
        Fx[0, 3] = cos_t * dt
        Fx[0, 4] = -sin_t * dt
        Fx[1, 2] = (vx * cos_t - vy * sin_t) * dt
        Fx[1, 3] = sin_t * dt
        Fx[1, 4] = cos_t * dt
        Fx[2, 5] = dt

        # Prediksi kovariansi: Sigma_{t|t-1} = F_x * Sigma * F_x^T + Q_t
        self.Sigma = Fx @ self.Sigma @ Fx.T + (self.Q_rate * dt)

    def _ekf_measurement_update(self, z: np.ndarray, H: np.ndarray, R: np.ndarray, angle_idx: int = -1):
        """
        Langkah 2 EKF: Measurement Update ("See") menggunakan Kalman Gain K_t
        sesuai Persamaan 5.49 - 5.51 Buku Siegwart.
        """
        # Inovasi (selisih pengukuran aktual z dengan prediksi H * x)
        innovation = z - (H @ self.state)
        if angle_idx >= 0:
            innovation[angle_idx] = normalize_angle(float(innovation[angle_idx]))

        # Inovasi Kovariansi: S_t = H * Sigma * H^T + R_t
        S = H @ self.Sigma @ H.T + R

        # Kalman Gain: K_t = Sigma * H^T * S_t^(-1)
        K = self.Sigma @ H.T @ np.linalg.inv(S)

        # Koreksi State & Kovariansi:
        self.state = self.state + (K @ innovation)
        self.state[2] = normalize_angle(float(self.state[2]))
        self.Sigma = (np.eye(6) - K @ H) @ self.Sigma

    def wheel_callback(self, msg: Odometry):
        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._predict_to_time(now_sec)

        # Pengukuran dari Encoder Roda: z_wheel = [vx, vy]^T
        z_wheel = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y], dtype=np.float64)
        H_wheel = np.zeros((2, 6), dtype=np.float64)
        H_wheel[0, 3] = 1.0  # vx
        H_wheel[1, 4] = 1.0  # vy
        R_wheel = np.diag([0.02, 0.02])  # Ketidakpastian roda omni

        self._ekf_measurement_update(z_wheel, H_wheel, R_wheel)
        self._publish_fused_odometry(msg.header.stamp)

    def imu_callback(self, msg: Imu):
        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._predict_to_time(now_sec)

        # D455 asli: hanya gyro (orientation_covariance[0] = -1, REP-145)
        # -> ukur wz saja; yaw EKF murni hasil integrasi (ikut drift bias gyro)
        if msg.orientation_covariance[0] < 0.0:
            z_g = np.array([msg.angular_velocity.z], dtype=np.float64)
            H_g = np.zeros((1, 6), dtype=np.float64)
            H_g[0, 5] = 1.0
            r_g = max(float(msg.angular_velocity_covariance[8]), 1e-6)
            self._ekf_measurement_update(z_g, H_g, np.array([[r_g]]))
            return

        # Pengukuran dari IMU dengan orientasi: z_imu = [theta, wz]^T
        yaw = quaternion_to_yaw(msg.orientation)
        z_imu = np.array([yaw, msg.angular_velocity.z], dtype=np.float64)
        H_imu = np.zeros((2, 6), dtype=np.float64)
        H_imu[0, 2] = 1.0  # theta
        H_imu[1, 5] = 1.0  # wz
        R_imu = np.diag([0.0005, 0.001])  # IMU sangat dipercaya untuk orientasi

        self._ekf_measurement_update(z_imu, H_imu, R_imu, angle_idx=0)

    def vo_callback(self, msg: Odometry):
        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._predict_to_time(now_sec)

        # Pengukuran dari RGB-D Visual Odometry D455: z_vo = [vx_vo, vy_vo]^T
        sigma_vo_sq = float(msg.twist.covariance[0])
        if sigma_vo_sq > 10.0:
            # Jika kamera menghadap tembok polos (fitur sedikit), abaikan update VO
            return

        z_vo = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y], dtype=np.float64)
        H_vo = np.zeros((2, 6), dtype=np.float64)
        H_vo[0, 3] = 1.0  # vx
        H_vo[1, 4] = 1.0  # vy
        R_vo = np.diag([sigma_vo_sq, sigma_vo_sq])

        self._ekf_measurement_update(z_vo, H_vo, R_vo)

    def _publish_fused_odometry(self, stamp):
        x, y, th, vx, vy, wz = self.state
        q_fused = euler_to_quaternion(float(th))

        # 1. Siarkan TF Dinamis hasil fusi EKF: odom -> base_link
        t_odom = TransformStamped()
        t_odom.header.stamp = stamp
        t_odom.header.frame_id = 'odom'
        t_odom.child_frame_id = 'base_link'
        t_odom.transform.translation.x = float(x)
        t_odom.transform.translation.y = float(y)
        t_odom.transform.translation.z = 0.0
        t_odom.transform.rotation = q_fused
        self.tf_broadcaster.sendTransform(t_odom)

        # 2. Publish topik /odom hasil fusi EKF beserta matriks kovariansi posterior Sigma_t
        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = float(x)
        odom_msg.pose.pose.position.y = float(y)
        odom_msg.pose.pose.orientation = q_fused

        cov_pose = [0.0] * 36
        cov_pose[0] = float(self.Sigma[0, 0])
        cov_pose[7] = float(self.Sigma[1, 1])
        cov_pose[14] = 1e-6
        cov_pose[21] = 1e-6
        cov_pose[28] = 1e-6
        cov_pose[35] = float(self.Sigma[2, 2])
        odom_msg.pose.covariance = cov_pose

        odom_msg.twist.twist.linear.x = float(vx)
        odom_msg.twist.twist.linear.y = float(vy)
        odom_msg.twist.twist.angular.z = float(wz)
        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EkfSensorFusionNode()
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
