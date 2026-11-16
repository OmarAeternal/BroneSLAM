#!/usr/bin/env python3
"""
Model sensor Intel RealSense D455 ASLI untuk simulasi Webots.

Sumber angka:
  [UJI] = hasil ukur kamera asli 2026-11-09 (serial 135122251358),
          ~/brone/D455_SPEK_ASLI/SPESIFIKASI_D455.md
  [DS]  = datasheet Intel RealSense D400 series
  [ASUMSI] = tidak diukur, dipilih wajar

Webots hanya merender citra "sempurna". Modul ini mengubahnya menjadi citra
yang mirip kamera asli:
  depth : noise ~Z^2, kuantisasi 1 mm, jarak minimum 0.293 m, bayangan stereo
          (termasuk pita kosong di sisi kiri), piksel hilang acak,
          lalu di-align ke frame kamera warna (seperti align_depth di driver asli)
  IMU   : gyro dengan bias + noise, accel dengan noise + skala |g|,
          TANPA orientasi absolut
  rate  : profil "usb3" (datasheet) atau "usb2" (persis seperti hasil uji)
"""

import math
import os
import numpy as np

# ---------------------------------------------------------------------------
# 1. Intrinsik & ekstrinsik [UJI] (640x480)
# ---------------------------------------------------------------------------
DEPTH_FX = 387.070          # FOV-H 79.16 deg
COLOR_FX = 379.907          # FOV-H 80.21 deg (fy asli 379.453, beda 0.12 %)
# cx/cy asli: depth (319.135, 237.717), color (318.194, 246.605).
# Webots selalu merender titik pusat di tengah gambar, jadi camera_info yang
# dipublish memakai pusat render ((w-1)/2, (h-1)/2) agar konsisten dengan citra.
REAL_DEPTH_CXCY = (319.135, 237.717)
REAL_COLOR_CXCY = (318.194, 246.605)
COLOR_DISTORTION = [-0.058852, 0.071007, -0.001030, -0.000151, -0.022689]  # tidak dirender

BASELINE_M = 0.09497                          # [UJI] baseline stereo
T_DEPTH_TO_COLOR = np.array([-0.05912, 0.00015, 0.00058])   # [UJI] frame optik
T_DEPTH_TO_IMU = np.array([-0.03022, 0.00740, 0.01602])     # [UJI] frame optik

# ---------------------------------------------------------------------------
# 2. Depth [UJI]
# ---------------------------------------------------------------------------
DEPTH_MIN_M = 0.293          # jarak terdekat terbaca; lebih dekat -> 0
DEPTH_MAX_M = 6.0            # [DS] batas efektif indoor (dipotong driver)
DEPTH_UNIT_M = 0.001         # Z16, 1 mm
# Noise temporal (std) vs jarak. Uji: 1.0 / 10.8 / 26.4 / 41.0 mm untuk rentang
# 0.3-1 / 1-2 / 2-3 / 3-4 m. Simpul di tengah rentang dikalibrasi (x~1.25) supaya
# median std per rentang di simulasi = angka uji (dicek dengan tools/cek_sensor_d455.py).
_NOISE_Z = np.array([0.30, 0.65, 1.50, 2.50, 3.50])
_NOISE_STD = np.array([0.0006, 0.00125, 0.0135, 0.0325, 0.0400])
DEPTH_DROPOUT_PROB = 0.01    # [ASUMSI] piksel hilang acak (speckle)

# ---------------------------------------------------------------------------
# 3. IMU [UJI] -- dikonversi dari frame optik (x kanan, y bawah, z depan)
#    ke frame camera_link (x depan, y kiri, z atas):  (x,y,z)_cl = (z, -x, -y)_opt
# ---------------------------------------------------------------------------
def _opt_to_cl(v):
    return np.array([v[2], -v[0], -v[1]], dtype=np.float64)

GYRO_BIAS = _opt_to_cl([-0.00185, 0.00089, -0.00034])        # rad/s
GYRO_NOISE_STD = np.abs(_opt_to_cl([0.0025, 0.0027, 0.0015]))  # rad/s
ACCEL_NOISE_STD = np.abs(_opt_to_cl([0.0076, 0.0083, 0.0087]))  # m/s^2
ACCEL_SCALE = 9.709 / 9.78   # |g| terukur 0.7 % di bawah g lokal

# ---------------------------------------------------------------------------
# 4. Profil laju data
# ---------------------------------------------------------------------------
PROFILES = {
    # Datasheet / USB 3: 640x480 @30 FPS, gyro 200 Hz, accel 100 Hz
    'usb3': dict(cam_hz=30.0, cam_drop=0.0, gyro_hz=200.0, accel_hz=100.0),
    # Hasil uji USB 2.1: diminta 15 FPS -> dapat 11.6; gyro 21.6 Hz, accel 12.2 Hz
    # (laju dasar di sim ~14.4 Hz karena langkah 16 ms; drop 0.20 -> ~11.6 Hz, dicek)
    'usb2': dict(cam_hz=15.0, cam_drop=0.20, gyro_hz=21.6, accel_hz=12.2),
}


def get_profile_name() -> str:
    name = os.environ.get('D455_PROFILE', 'usb3').strip().lower()
    if name not in PROFILES:
        raise ValueError(f"D455_PROFILE='{name}' tidak dikenal, pilih: {list(PROFILES)}")
    return name


def realism_enabled() -> bool:
    """D455_REALISM=0 mematikan semua noise/artefak (citra ideal, intrinsik tetap asli)."""
    return os.environ.get('D455_REALISM', '1') != '0'


class RateGate:
    """Memutuskan apakah frame pada waktu sim t dipublish (laju + frame drop acak)."""

    def __init__(self, hz: float, drop_prob: float, rng: np.random.Generator):
        self.period = 1.0 / hz
        self.drop = drop_prob
        self.rng = rng
        self.next_t = None

    def ready(self, t: float) -> bool:
        if self.next_t is None:
            self.next_t = t
        if t + 1e-9 < self.next_t:
            return False
        # jadwal berikutnya (tidak menumpuk kalau sim lebih lambat)
        self.next_t = max(self.next_t + self.period, t)
        if self.drop > 0.0 and self.rng.random() < self.drop:
            return False
        return True


class D455DepthModel:
    def __init__(self, width: int, height: int, fx_depth: float, fx_color: float,
                 seed: int = 455, enabled: bool = True):
        self.w, self.h = width, height
        self.enabled = enabled
        self.rng = np.random.default_rng(seed)
        self.fx_d = fx_depth
        self.fx_c = fx_color
        self.cx = (width - 1) / 2.0
        self.cy = (height - 1) / 2.0
        uu, vv = np.meshgrid(np.arange(width, dtype=np.float32),
                             np.arange(height, dtype=np.float32))
        self.xn = (uu - self.cx) / self.fx_d    # sinar ternormalisasi depth
        self.yn = (vv - self.cy) / self.fx_d
        self.u_idx = uu

    # -- noise std sebagai fungsi Z (interpolasi data uji, >3.5 m naik ~Z^2) --
    @staticmethod
    def noise_std(z: np.ndarray) -> np.ndarray:
        s = np.interp(z, _NOISE_Z, _NOISE_STD)
        far = z > _NOISE_Z[-1]
        s[far] = _NOISE_STD[-1] * (z[far] / _NOISE_Z[-1]) ** 2
        return s

    def degrade(self, z_true: np.ndarray) -> np.ndarray:
        """Citra depth Webots (m, float32, planar Z) -> citra depth seperti D455 asli (0 = kosong)."""
        z = z_true.astype(np.float32, copy=True)
        z[~np.isfinite(z)] = 0.0
        z[z >= DEPTH_MAX_M] = 0.0   # di luar jangkauan = kosong (Webots memberi maxRange/inf)

        if not self.enabled:
            z[z < DEPTH_MIN_M] = 0.0
            return z

        valid = z > 0.0
        # (a) bayangan stereo: piksel imager kiri (referensi) yang tidak terlihat imager kanan.
        #     disparitas d = fx*B/Z; posisi di imager kanan x_r = u - d.
        #     Piksel terlihat jika x_r >= 0 dan tidak tertutup piksel lain di kanannya
        #     (x_r harus < minimum x_r semua piksel di sebelah kanannya).
        #     Ini otomatis menghasilkan pita kosong ~30 px di kiri gambar (Z~1.2 m) seperti hasil uji.
        disp = np.zeros_like(z)
        disp[valid] = (self.fx_d * BASELINE_M) / z[valid]
        x_r = np.where(valid, self.u_idx - disp, np.inf).astype(np.float32)
        right_min = np.minimum.accumulate(x_r[:, ::-1], axis=1)[:, ::-1]
        right_min = np.concatenate([right_min[:, 1:], np.full((self.h, 1), np.inf, np.float32)], axis=1)
        visible = (x_r >= 0.0) & (x_r < right_min + 0.5)

        # (b) noise temporal ~ Z^2
        std = self.noise_std(z)
        z = z + self.rng.standard_normal(z.shape, dtype=np.float32) * std.astype(np.float32)

        # (c) kuantisasi Z16 1 mm
        z = np.round(z / DEPTH_UNIT_M) * DEPTH_UNIT_M

        # (d) piksel hilang acak
        drop = self.rng.random(z.shape, dtype=np.float32) < DEPTH_DROPOUT_PROB

        keep = valid & visible & ~drop & (z >= DEPTH_MIN_M) & (z < DEPTH_MAX_M)
        z[~keep] = 0.0
        return z.astype(np.float32)

    def align_to_color(self, z_depth: np.ndarray) -> np.ndarray:
        """Proyeksikan depth (frame imager kiri) ke frame kamera warna (seperti rs align_depth)."""
        valid = z_depth > 0.0
        z = z_depth[valid]
        X = self.xn[valid] * z + T_DEPTH_TO_COLOR[0]
        Y = self.yn[valid] * z + T_DEPTH_TO_COLOR[1]
        Zc = z + T_DEPTH_TO_COLOR[2]
        uc = np.rint(self.fx_c * X / Zc + self.cx).astype(np.int32)
        vc = np.rint(self.fx_c * Y / Zc + self.cy).astype(np.int32)
        inb = (uc >= 0) & (uc < self.w) & (vc >= 0) & (vc < self.h)
        flat = vc[inb] * self.w + uc[inb]
        out = np.full(self.w * self.h, np.inf, dtype=np.float32)
        np.minimum.at(out, flat, Zc[inb].astype(np.float32))   # z-buffer: titik terdekat menang
        out[~np.isfinite(out)] = 0.0
        return out.reshape(self.h, self.w)


class D455ImuModel:
    """Gyro + accel BMI085. Tidak ada orientasi; bias gyro konstan (tidak dikoreksi)."""

    def __init__(self, seed: int = 85, enabled: bool = True):
        self.rng = np.random.default_rng(seed)
        self.enabled = enabled

    def gyro(self, w_true) -> np.ndarray:
        w = np.asarray(w_true, dtype=np.float64)
        if not self.enabled:
            return w
        return w + GYRO_BIAS + self.rng.standard_normal(3) * GYRO_NOISE_STD

    def accel(self, a_true) -> np.ndarray:
        a = np.asarray(a_true, dtype=np.float64)
        if not self.enabled:
            return a
        return a * ACCEL_SCALE + self.rng.standard_normal(3) * ACCEL_NOISE_STD
