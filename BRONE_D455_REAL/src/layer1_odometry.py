#!/usr/bin/env python3
"""
LAYER 1: Proprioceptive Motion Model (u_t -> x_hat_t)
Referensi: Siegwart et al., Bab 5.2.4 (Odometry Error Model) & Figure 10.2

Tugas modul ini:
1. Inverse Kinematics: Mengubah perintah kecepatan robot (vx, vy, wz) menjadi
   kecepatan putar 3 roda omni (w1, w2, w3).
2. Forward Kinematics + IMU Fusion + Anti-Slip Guard: Membaca perubahan sudut
   3 encoder roda (delta_phi_1..3) untuk menghitung perpindahan translasi lokal (dx_r, dy_r),
   menggabungkannya dengan orientasi Yaw (theta) dari IMU D455, serta menolak
   selip roda 100% (wheel burnout) ketika robot tertahan tembok/objek (`is_stalled=True`).
3. Menyediakan Matriks Kovariansi Odometri (Q_t).
"""

import math
from typing import Tuple, List


class OmniImuOdometry:
    def __init__(self, wheel_radius: float = 0.05, body_radius: float = 0.12):
        self.r = wheel_radius   # Jari-jari roda omni = 0.05 m
        self.L = body_radius    # Jarak titik pusat robot ke roda = 0.12 m

        # State estimasi pose robot [x, y, theta] di kerangka 'odom'
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.prev_phi = [None, None, None]
        self.init_yaw = None

        # Matriks Kovariansi Odometri (Q_t)
        self.sigma_x_sq = 0.02       # Variansi posisi X (m^2)
        self.sigma_y_sq = 0.02       # Variansi posisi Y (m^2)
        self.sigma_theta_sq = 0.001  # Variansi rotasi Yaw dari IMU (rad^2)

    def inverse_kinematics(self, vx: float, vy: float, wz: float) -> Tuple[float, float, float]:
        """
        Mengubah kecepatan badan robot (vx, vy, wz) menjadi kecepatan sudut
        ketiga motor roda omni (w1, w2, w3) dalam rad/s.
        """
        sqrt3_2 = math.sqrt(3.0) / 2.0

        w1 = (vy - self.L * wz) / self.r
        w2 = (-sqrt3_2 * vx - 0.5 * vy - self.L * wz) / self.r
        w3 = (sqrt3_2 * vx - 0.5 * vy - self.L * wz) / self.r

        return w1, w2, w3

    def update(
        self,
        phi1: float,
        phi2: float,
        phi3: float,
        imu_yaw: float,
        dt: float,
        is_stalled: bool = False
    ) -> Tuple[float, float, float, float, float, float]:
        """
        Menghitung pose baru (x, y, theta) dan kecepatan aktual (vx, vy, wz)
        dari input proprioseptif u_t = (encoder roda + IMU yaw).
        Jika `is_stalled=True` (kamera D455 mendeteksi pemandangan diam total
        saat robot menabrak tembok), maka translasi selip roda ditolak (dx_r = dy_r = 0).
        """
        if self.prev_phi[0] is None or self.init_yaw is None:
            self.prev_phi = [phi1, phi2, phi3]
            self.init_yaw = imu_yaw
            return self.x, self.y, self.theta, 0.0, 0.0, 0.0

        # 1. Hitung perubahan sudut putaran masing-masing roda (rad)
        dphi1 = phi1 - self.prev_phi[0]
        dphi2 = phi2 - self.prev_phi[1]
        dphi3 = phi3 - self.prev_phi[2]
        self.prev_phi = [phi1, phi2, phi3]

        # 2. Konversi perubahan sudut menjadi jarak busur roda: ds_i = r * dphi_i
        ds1 = self.r * dphi1
        ds2 = self.r * dphi2
        ds3 = self.r * dphi3

        # 3. Forward Kinematics: Perpindahan translasi di kerangka lokal robot (base_link)
        dx_r = (ds3 - ds2) / math.sqrt(3.0)
        dy_r = (2.0 * ds1 - ds2 - ds3) / 3.0

        # 4. Orientasi (theta) dari IMU D455
        new_theta = self._normalize_angle(imu_yaw - self.init_yaw)
        dtheta = self._normalize_angle(new_theta - self.theta)

        # 5. Proteksi Anti-Slip (Visual Stall Rejection):
        # Jika robot tidak sedang berputar (|dtheta| kecil) tetapi kamera D455 melihat
        # tembok/objek di depan sama sekali tidak bergerak (is_stalled=True),
        # berarti roda sedang selip menggerus lantai karena menabrak!
        if is_stalled and abs(dtheta) < 0.003:
            dx_r = 0.0
            dy_r = 0.0

        mid_theta = self.theta + 0.5 * dtheta
        self.x += dx_r * math.cos(mid_theta) - dy_r * math.sin(mid_theta)
        self.y += dx_r * math.sin(mid_theta) + dy_r * math.cos(mid_theta)
        self.theta = new_theta

        vx = dx_r / dt if dt > 1e-5 else 0.0
        vy = dy_r / dt if dt > 1e-5 else 0.0
        wz = dtheta / dt if dt > 1e-5 else 0.0

        return self.x, self.y, self.theta, vx, vy, wz

    def get_pose_covariance_6x6(self) -> List[float]:
        cov = [0.0] * 36
        cov[0] = self.sigma_x_sq
        cov[7] = self.sigma_y_sq
        cov[14] = 1e-6
        cov[21] = 1e-6
        cov[28] = 1e-6
        cov[35] = self.sigma_theta_sq
        return cov

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
