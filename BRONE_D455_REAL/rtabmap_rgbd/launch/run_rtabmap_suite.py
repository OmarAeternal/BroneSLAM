#!/usr/bin/env python3
"""
Master Launcher: EKF Sensor Fusion + RTAB-Map RGB-D Visual SLAM Suite
*** VERSI SENSOR D455 SPEK ASLI (BRONE_D455_REAL) ***

Opsi (environment variable):
  D455_PROFILE=usb3|usb2   laju data datasheet USB3 (default) / hasil uji USB 2.1
  D455_REALISM=1|0         noise & artefak kamera asli ON (default) / OFF
  D455_BRAKE_DIST=0.40     jarak rem anti-tabrak (m)
  NO_RVIZ=1                tanpa RViz2
=============================================================================
Menjalankan arsitektur lengkap Bab 5.6 (EKF) & Bab 5.8.5 (RTAB-Map RGB-D):
  1. Webots Simulator (worlds/openrmf_warehouse.wbt)
  2. d455_rtabmap_driver.py (Ekstraktor D455 RGB + Depth 32FC1 + CameraInfo K + /wheel/odom + /imu/data)
  3. rgbd_visual_odometry.py (Layer 1.5A: Optical Flow + 3D Back-Projection -> /vo/odom)
  4. ekf_sensor_fusion.py (Layer 1.5B: Extended Kalman Filter 6-State -> /odom + TF odom->base_link)
  5. RTAB-Map SLAM Node (ros2 run rtabmap_slam rtabmap dengan --params-file rtabmap_params.yaml)
  6. RViz2 dengan konfigurasi rtabmap_view.rviz

Cara menjalankan:
  python3 ~/brone/BRONE_D455_REAL/rtabmap_rgbd/launch/run_rtabmap_suite.py
"""

import os
import sys
import time
import signal
import shutil
import subprocess

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
RTAB_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

WORLD_FILE = os.path.join(BASE_DIR, 'worlds', 'openrmf_warehouse.wbt')
DRIVER_NODE = os.path.join(RTAB_DIR, 'src', 'd455_rtabmap_driver.py')
VO_NODE = os.path.join(RTAB_DIR, 'src', 'rgbd_visual_odometry.py')
EKF_NODE = os.path.join(RTAB_DIR, 'src', 'ekf_sensor_fusion.py')
FALLBACK_SLAM_NODE = os.path.join(BASE_DIR, 'src', 'layer3_graph_slam.py')
RTAB_PARAMS = os.path.join(RTAB_DIR, 'config', 'rtabmap_params.yaml')
RVIZ_CONFIG = os.path.join(RTAB_DIR, 'config', 'rtabmap_view.rviz')
# Semua node memakai waktu simulasi dari /clock (dipublish driver)
SIM_TIME = ['--ros-args', '-p', 'use_sim_time:=true']


def check_rtabmap_installed():
    """Cek apakah paket ros2 rtabmap_slam tersedia di sistem."""
    try:
        res = subprocess.run(
            ['ros2', 'pkg', 'prefix', 'rtabmap_slam'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=4
        )
        return res.returncode == 0
    except Exception:
        return False


def main():
    procs = []

    # Setiap proses dijalankan di process-group sendiri, lalu dimatikan per GROUP.
    # Tanpa ini, 'ros2 run' mati tapi binary rtabmap di dalamnya tetap hidup (yatim).
    def _signal_all(sig):
        for p in reversed(procs):
            try:
                os.killpg(p.pid, sig)
            except Exception:
                pass

    def cleanup(sig=None, frame=None):
        print("\n[SHUTDOWN] Menghentikan seluruh proses EKF + RTAB-Map Suite...")
        _signal_all(signal.SIGINT)
        time.sleep(2.0)
        _signal_all(signal.SIGTERM)
        time.sleep(1.0)
        _signal_all(signal.SIGKILL)
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    print("=" * 74)
    print("  BRONE D455 REAL: EKF + RTAB-MAP RGB-D SLAM (sensor = spek D455 asli)")
    print(f"  Profil={os.environ.get('D455_PROFILE', 'usb3')}  Realisme={os.environ.get('D455_REALISM', '1')}  "
          f"Rem={os.environ.get('D455_BRAKE_DIST', '0.40')} m")
    print("  Referensi: Roland Siegwart, Bab 5.6 (EKF) & Bab 5.8.5 (RGB-D SLAM)")
    print("=" * 74)

    # 1. Jalankan Webots
    webots_bin = shutil.which('webots')
    if not webots_bin:
        print("[ERROR] Binary 'webots' tidak ditemukan di PATH!")
        sys.exit(1)

    print(f"[1/6] Menjalankan Webots: {WORLD_FILE}")
    webots_args = ['--minimize'] if os.environ.get('WEBOTS_MINIMIZE') == '1' else []
    p_webots = subprocess.Popen([webots_bin] + webots_args + [WORLD_FILE], start_new_session=True)
    procs.append(p_webots)
    time.sleep(3.5)

    # 2. Jalankan D455 RTAB-Map Driver
    print("[2/6] Menjalankan Layer 0-2 D455 RGB-D Driver (d455_rtabmap_driver.py)...")
    p_driver = subprocess.Popen(['python3', DRIVER_NODE], start_new_session=True)
    procs.append(p_driver)
    time.sleep(1.5)

    # 3. Jalankan RGB-D Visual Odometry (Layer 1.5A)
    print("[3/6] Menjalankan Layer 1.5A RGB-D Visual Odometry (rgbd_visual_odometry.py)...")
    p_vo = subprocess.Popen(['python3', VO_NODE] + SIM_TIME, start_new_session=True)
    procs.append(p_vo)
    time.sleep(0.5)

    # 4. Jalankan Extended Kalman Filter (Layer 1.5B)
    print("[4/6] Menjalankan Layer 1.5B Extended Kalman Filter (ekf_sensor_fusion.py)...")
    p_ekf = subprocess.Popen(['python3', EKF_NODE] + SIM_TIME, start_new_session=True)
    procs.append(p_ekf)
    time.sleep(1.0)

    # 5. Jalankan RTAB-Map SLAM (menggunakan --params-file agar tipe parameter string terjaga)
    has_rtabmap = check_rtabmap_installed()
    if has_rtabmap:
        print("[5/6] Paket 'rtabmap_slam' ditemukan! Menjalankan RTAB-Map RGB-D + ICP SLAM...")
        rtabmap_cmd = [
            'ros2', 'run', 'rtabmap_slam', 'rtabmap',
            '--ros-args',
            '--params-file', RTAB_PARAMS,
            '-r', 'rgb/image:=/camera/color/image_raw',
            '-r', 'depth/image:=/camera/aligned_depth_to_color/image_raw',
            '-r', 'rgb/camera_info:=/camera/color/camera_info',
            '-r', 'scan:=/scan',
            '-r', 'odom:=/odom',
            '-r', 'grid_map:=/map',
            '-r', 'cloud_map:=/rtabmap/cloud_map',
            '-r', 'mapPath:=/rtabmap/mapPath',
            '--', '--delete_db_on_start'
        ]
        p_slam = subprocess.Popen(rtabmap_cmd, start_new_session=True)
        procs.append(p_slam)
    else:
        print("[5/6] Catatan: Paket apt 'ros-$ROS_DISTRO-rtabmap-slam' belum terinstal.")
        print("      -> Menjalankan Built-in Pose-Graph SLAM di atas EKF (/odom) sebagai fallback...")
        p_slam = subprocess.Popen(['python3', FALLBACK_SLAM_NODE] + SIM_TIME, start_new_session=True)
        procs.append(p_slam)

    # 6. Jalankan RViz2
    rviz_bin = None if os.environ.get('NO_RVIZ') == '1' else shutil.which('rviz2')
    if rviz_bin:
        print(f"[6/6] Menjalankan RViz2: {RVIZ_CONFIG}")
        p_rviz = subprocess.Popen([rviz_bin, '-d', RVIZ_CONFIG] + SIM_TIME, start_new_session=True)
        procs.append(p_rviz)

    print("\n" + "=" * 74)
    print("  SELURUH LAYER AKTIF! Buka terminal baru untuk menggerakkan robot:")
    print(f"  python3 {os.path.join(BASE_DIR, 'src', 'teleop_explorer.py')}")
    print("  Atau jalan otomatis memetakan ruangan (frontier / random):")
    print(f"  python3 {os.path.join(BASE_DIR, 'src', 'auto_explorer.py')} [--mode random]")
    print(f"  Ke kotak grid (lihat {os.path.join(BASE_DIR, 'peta_grid', 'peta_grid.png')}):")
    print(f"  python3 {os.path.join(BASE_DIR, 'src', 'auto_explorer.py')} --2c")
    print("  Cek sensor (bandingkan dengan kamera asli):")
    print(f"  python3 {os.path.join(BASE_DIR, 'tools', 'cek_sensor_d455.py')}")
    print("=" * 74 + "\n")

    while True:
        time.sleep(1.0)
        if p_webots.poll() is not None:
            print("[INFO] Webots ditutup oleh user. Menghentikan seluruh node...")
            cleanup()


if __name__ == '__main__':
    main()
