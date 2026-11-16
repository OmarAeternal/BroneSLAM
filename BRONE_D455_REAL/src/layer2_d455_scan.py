#!/usr/bin/env python3
"""
LAYER 2: Exteroceptive Observation Model (z_t) - Pure Intel RealSense D455
Referensi:
- Siegwart et al., Bab 5.8 & Bab 4.1 (Pinhole Camera & Range Perception)
- Intel RealSense D400 Series Datasheet (Tabel 3-42, hal. 55):
  * Depth HFOV = 86 deg (1.5010 rad)
  * Resolusi = 640 x 480
  * Rentang Efektif = 0.4 m s/d 6.0 m

Perbaikan Proteksi Blind-Zone & Anti-Slip (Hasil Diskusi Bug Tembok):
1. Proteksi OpenGL zNear Clipping (< 0.40m):
   Jika robot berada sangat dekat dengan tembok (0.05m <= Z < 0.40m), nilai jarak
   TIDAK BOLEH diubah menjadi 'inf' (ruang kosong), melainkan dikunci sebagai
   jarak dekat aktual agar tembok di RViz2 tidak terhapus!
2. Deteksi Visual Stall (Stationary Scene Detection dari D455 Depth):
   Membandingkan perubahan matriks kedalaman D455 antar-frame (|Z_t - Z_{t-1}|).
   Jika roda berputar tetapi citra kedalaman D455 sama sekali tidak berubah,
   berarti robot sedang tertahan tembok/meja (100% wheel slip)!
"""

import math
from typing import List, Tuple
import numpy as np


class D455DepthToScanConverter:
    def __init__(
        self,
        width: int = 640,
        height: int = 480,
        hfov_rad: float = 1.5010,   # 86.0 derajat (Datasheet D455 Tabel 3-42)
        range_min: float = 0.12,    # Batas baca dekat aman agar tembok dekat tidak hilang
        range_max: float = 6.00,    # Max-Z efektif D455 indoor (m)
        num_beams: int = 160        # Jumlah sinar virtual horizontal dari 640 piksel D455
    ):
        self.width = width
        self.height = height
        self.hfov = hfov_rad
        self.range_min = max(0.12, float(range_min))
        self.range_max = float(range_max)
        self.num_beams = num_beams

        # Parameter sudut sesuai standar ROS REP-103:
        # angle_min = -43 deg (kanan kamera), angle_max = +43 deg (kiri kamera)
        self.angle_min = -self.hfov / 2.0
        self.angle_max = self.hfov / 2.0
        self.angle_increment = self.hfov / float(self.num_beams - 1)

        # Intrinsik kamera Pinhole D455 (panjang fokus horizontal fx dalam piksel)
        self.cx = (self.width - 1) / 2.0
        self.fx = (self.width / 2.0) / math.tan(self.hfov / 2.0)

        # Pre-compute pemetaan indeks sudut alpha_i -> kolom piksel u_i dan faktor 1/cos(alpha_i)
        self.pixel_cols = []
        self.inv_cos = []
        for i in range(self.num_beams):
            alpha = self.angle_min + i * self.angle_increment
            u = int(round(self.cx - self.fx * math.tan(alpha)))
            u = max(0, min(self.width - 1, u))
            self.pixel_cols.append(u)
            self.inv_cos.append(1.0 / math.cos(alpha))

        self.prev_mid_band = None

    def analyze_depth_and_extract_scan(
        self, depth_buffer: bytes
    ) -> Tuple[List[float], float, bool]:
        """
        Mengekstrak:
        1. ranges: Array jarak radial R untuk sensor_msgs/LaserScan.
        2. front_min_dist: Jarak objek terdekat di depan robot (untuk Depth Bumper).
        3. is_scene_stationary: True jika gambar Depth D455 diam total (deteksi roda selip saat nabrak).
        """
        depth_img = np.frombuffer(depth_buffer, dtype=np.float32).reshape((self.height, self.width))

        # Ambil pita horizontal setinggi badan robot (baris 220 s/d 260)
        mid_row = self.height // 2
        band = depth_img[mid_row - 15:mid_row + 15, ::4]

        # 1. Deteksi apakah pandangan D455 diam total (Visual Stall Check)
        is_scene_stationary = False
        if self.prev_mid_band is not None:
            valid_mask = np.isfinite(band) & np.isfinite(self.prev_mid_band) & (band < self.range_max)
            if np.count_nonzero(valid_mask) > 50:
                median_diff = float(np.median(np.abs(band[valid_mask] - self.prev_mid_band[valid_mask])))
                # Jika perubahan median depth < 0.4 mm, berarti bodi robot sebenarnya diam tertahan tembok!
                if median_diff < 0.0004:
                    is_scene_stationary = True
        self.prev_mid_band = band.copy()

        # 2. Jarak terdekat di kerucut depan kamera (+/- 25 derajat tengah)
        center_slice = depth_img[mid_row - 20:mid_row + 20, self.width // 4: 3 * self.width // 4]
        valid_center = center_slice[np.isfinite(center_slice) & (center_slice > 0.04)]
        front_min_dist = float(np.min(valid_center)) if valid_center.size > 0 else float('inf')

        # 3. Ekstraksi baris horizontal untuk /scan dengan koreksi Pinhole R = Z / cos(alpha)
        row_depths = depth_img[mid_row, :]
        ranges = []
        for u, sec_alpha in zip(self.pixel_cols, self.inv_cos):
            z_perp = float(row_depths[u])

            if math.isnan(z_perp) or math.isinf(z_perp):
                ranges.append(float('inf'))
                continue

            r_radial = z_perp * sec_alpha

            if r_radial < 0.05:
                ranges.append(float('nan'))
            elif r_radial < self.range_min:
                # Kunci ke range_min agar tembok yang sangat dekat (< 0.4m) TIDAK dianggap 'inf' (kosong)!
                ranges.append(self.range_min)
            elif r_radial >= self.range_max:
                ranges.append(float('inf'))
            else:
                ranges.append(r_radial)

        return ranges, front_min_dist, is_scene_stationary
