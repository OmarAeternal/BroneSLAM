#!/usr/bin/env python3
"""
Cek sensor D455 simulasi dan bandingkan dengan hasil ukur kamera asli (2026-11-09).
Pengukurannya sama dengan uji kamera asli: robot DIAM, rekam N detik, lalu hitung
FPS, intrinsik, piksel valid, jarak min, noise temporal per rentang jarak, bias/noise gyro, |g|.

Pakai:  python3 tools/cek_sensor_d455.py [detik]      (default 20 detik)
Robot harus diam (jangan jalankan teleop/auto-explore selama pengukuran).
"""

import math
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu

ASLI = {  # hasil uji kamera asli (USB 2.1, diminta 15 FPS)
    'fps_depth': '11.6 (USB2) / 30 (USB3, datasheet)',
    'fx_depth': 387.070, 'fov_depth': 79.16, 'fx_color': 379.907,
    'valid': 61.3, 'min': 0.293,
    'noise': {(0.3, 1): 1.0, (1, 2): 10.8, (2, 3): 26.4, (3, 4): 41.0},
    'gyro_bias_z_cl': -0.00089, 'gyro_std_z_cl': 0.0027, 'g': 9.709,
}


class Cek(Node):
    def __init__(self):
        super().__init__('cek_sensor_d455')
        self.t = {'color': [], 'depth_raw': [], 'aligned': [], 'imu': []}
        self.frames = []
        self.gyro, self.acc = [], []
        self.info = {}
        self.create_subscription(Image, '/camera/color/image_raw', lambda m: self._t('color'), 10)
        self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', lambda m: self._t('aligned'), 10)
        self.create_subscription(Image, '/camera/depth/image_rect_raw', self.depth_cb, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_cb, 50)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', lambda m: self.info.__setitem__('depth', m), 2)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', lambda m: self.info.__setitem__('color', m), 2)

    def _t(self, k):
        self.t[k].append(time.monotonic())

    def depth_cb(self, m):
        self._t('depth_raw')
        if len(self.frames) < 300:
            self.frames.append(np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width).copy())

    def imu_cb(self, m):
        self._t('imu')
        self.gyro.append([m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
        self.acc.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])


def hz(ts):
    if len(ts) < 2:
        return 0.0, 0.0
    d = np.diff(ts)
    return (len(ts) - 1) / (ts[-1] - ts[0]), float(d.max())


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    rclpy.init()
    n = Cek()
    print(f'Merekam {dur:.0f} detik (wall-clock)... pastikan robot diam.')
    t_end = time.monotonic() + dur
    while time.monotonic() < t_end:
        rclpy.spin_once(n, timeout_sec=0.05)

    print('\n=========== HASIL: SIMULASI vs D455 ASLI ===========')
    for k in ['color', 'depth_raw', 'aligned', 'imu']:
        f, gap = hz(n.t[k])
        print(f'Laju {k:10s}: {f:6.1f} Hz (jeda maks {gap*1000:5.0f} ms)  [wall-clock]')
    print(f"  asli depth/color: {ASLI['fps_depth']}; IMU asli USB2 gyro 21.6 Hz / USB3 200 Hz")

    for key, fx_asli in [('depth', ASLI['fx_depth']), ('color', ASLI['fx_color'])]:
        if key in n.info:
            ci = n.info[key]
            fx = ci.k[0]
            fov = math.degrees(2 * math.atan(ci.width / 2 / fx))
            print(f'fx {key:5s}: sim {fx:8.3f} (FOV-H {fov:5.2f} deg) | asli {fx_asli:.3f}')

    if not n.frames:
        print('Tidak ada frame depth! Apakah simulasi berjalan?')
        return
    st = np.stack(n.frames)
    valid = st > 0
    allv = st[valid]
    print(f'\nFrame depth terekam : {len(st)}')
    print(f'Piksel valid rata2  : sim {valid.mean()*100:5.1f} % | asli {ASLI["valid"]} % (tergantung scene)')
    print(f'Jarak terdekat      : sim {allv.min():.3f} m | asli {ASLI["min"]} m')
    print(f'Persentil 1/50/99 % : {np.percentile(allv,1):.3f} / {np.percentile(allv,50):.3f} / {np.percentile(allv,99):.3f} m')
    left = valid[:, :, :40].mean(axis=(0, 1))
    first_ok = int(np.argmax(left > 0.5)) if (left > 0.5).any() else 40
    print(f'Pita kosong kiri    : ~{first_ok} px (asli ±30 px)')

    always = valid.all(axis=0)
    if len(st) >= 5 and always.any():
        mean = st.mean(axis=0)
        std = st.std(axis=0)
        print('\nNoise temporal (std per piksel, piksel yang selalu valid):')
        for (a, b), asli in ASLI['noise'].items():
            m = always & (mean >= a) & (mean < b)
            if m.sum() > 50:
                s = float(np.median(std[m])) * 1000
                print(f'  {a:.1f}-{b:.0f} m : sim {s:5.1f} mm | asli {asli:5.1f} mm   ({m.sum()} px)')
            else:
                print(f'  {a:.1f}-{b:.0f} m : (piksel terlalu sedikit di rentang ini) | asli {asli} mm')

    if len(n.gyro) > 10:
        g = np.array(n.gyro)
        a = np.array(n.acc)
        print('\nIMU (frame camera_imu_frame: x depan, y kiri, z atas):')
        print(f'  gyro mean (bias) : {g.mean(0)} rad/s')
        print(f'  gyro std         : {g.std(0)} rad/s')
        print(f'  asli yaw (z)     : bias {ASLI["gyro_bias_z_cl"]} rad/s, std {ASLI["gyro_std_z_cl"]} rad/s')
        print(f'  |g| accel        : sim {np.linalg.norm(a.mean(0)):.3f} | asli {ASLI["g"]} m/s^2')
        drift = math.degrees(g.mean(0)[2]) * 30
        print(f'  drift yaw/30 s   : {drift:+.2f} deg (asli sumbu-x optik ±3.2 deg, yaw ±1.5 deg)')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
