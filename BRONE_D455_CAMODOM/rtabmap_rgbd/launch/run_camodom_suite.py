#!/usr/bin/env python3
"""
Master Launcher: ODOMETRI KAMERA (RTAB-Map rgbd_odometry) + RTAB-Map RGB-D SLAM
*** BRONE_D455_CAMODOM: sensor = spek D455 asli, odometri TANPA encoder roda ***

Beda dengan BRONE_D455_REAL (run_rtabmap_suite.py):
  lama : encoder roda + gyro + VO LK (python) -> EKF -> /odom
  baru : RGB + depth D455 -> rtabmap_odom/rgbd_odometry (F2M + bundle adjustment) -> /odom
  /wheel/odom tetap dipublish driver, tapi TIDAK dipakai siapa pun.

Urutan:
  1. Webots (worlds/openrmf_warehouse.wbt)
  2. d455_rtabmap_driver.py  (RGB, depth ter-align, camera_info, /scan, /imu/data, /clock)
  3. d455_gyro_guess.py      (gyro D455 -> TF odom_gyro->base_link, tebakan rotasi)
     rgbd_odometry           (/odom + /odom_info + TF odom->odom_gyro)
     => TF odom->base_link = odometri kamera (RGB-D + gyro D455)
  4. rtabmap                 (/map + TF map->odom)
  5. RViz2

Opsi (environment variable):
  D455_PROFILE=usb3|usb2   laju data datasheet USB3 (default) / hasil uji USB 2.1
  D455_REALISM=1|0         noise & artefak kamera asli ON (default) / OFF
  D455_BRAKE_DIST=0.40     jarak rem anti-tabrak (m)
  CAMODOM_STRATEGY=f2m|f2f Frame-to-Map (default) / Frame-to-Frame
  CAMODOM_GYRO=1|0         pakai gyro D455 sebagai tebakan rotasi (default 1) /
                           visual murni (0; sering hilang saat berputar di scene polos)
  NO_RVIZ=1                tanpa RViz2
  WEBOTS_MINIMIZE=1        jendela Webots diperkecil
  WEBOTS_PORT=1235         jalankan Webots di port lain (bila ada Webots lain yang hidup;
                           pakai juga ROS_DOMAIN_ID berbeda supaya topik tidak tercampur)

Cara menjalankan:
  python3 ~/brone/BRONE_D455_CAMODOM/rtabmap_rgbd/launch/run_camodom_suite.py
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
GYRO_NODE = os.path.join(RTAB_DIR, 'src', 'd455_gyro_guess.py')
ODOM_PARAMS = os.path.join(RTAB_DIR, 'config', 'camera_odom_params.yaml')
RTAB_PARAMS = os.path.join(RTAB_DIR, 'config', 'rtabmap_params_camodom.yaml')
RVIZ_CONFIG = os.path.join(RTAB_DIR, 'config', 'rtabmap_view.rviz')
# Database terpisah dari BRONE_D455_REAL (~/.ros/rtabmap.db) agar tidak saling hapus
DB_PATH = os.path.expanduser('~/.ros/rtabmap_camodom.db')
SIM_TIME = ['--ros-args', '-p', 'use_sim_time:=true']

CAMERA_REMAP = [
    '-r', 'rgb/image:=/camera/color/image_raw',
    '-r', 'depth/image:=/camera/aligned_depth_to_color/image_raw',
    '-r', 'rgb/camera_info:=/camera/color/camera_info',
]


def ros_pkg_exists(pkg):
    try:
        res = subprocess.run(['ros2', 'pkg', 'prefix', pkg], stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, text=True, timeout=6)
        return res.returncode == 0
    except Exception:
        return False


def main():
    procs = []

    # Tiap proses di process-group sendiri, dimatikan per GROUP (agar binary di
    # dalam 'ros2 run' tidak tertinggal yatim).
    def _signal_all(sig):
        for p in reversed(procs):
            try:
                os.killpg(p.pid, sig)
            except Exception:
                pass

    def cleanup(sig=None, frame=None):
        print("\n[SHUTDOWN] Menghentikan seluruh proses Camera-Odom + RTAB-Map Suite...")
        _signal_all(signal.SIGINT)
        time.sleep(2.0)
        _signal_all(signal.SIGTERM)
        time.sleep(1.0)
        _signal_all(signal.SIGKILL)
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    strategy = os.environ.get('CAMODOM_STRATEGY', 'f2m').lower()
    strategy_id = '1' if strategy == 'f2f' else '0'
    use_gyro = os.environ.get('CAMODOM_GYRO', '1') == '1'

    print("=" * 74)
    print("  BRONE D455 CAMODOM: ODOMETRI KAMERA (rgbd_odometry) + RTAB-MAP RGB-D SLAM")
    print(f"  Profil={os.environ.get('D455_PROFILE', 'usb3')}  Realisme={os.environ.get('D455_REALISM', '1')}  "
          f"Rem={os.environ.get('D455_BRAKE_DIST', '0.40')} m  Odom={'F2F' if strategy_id == '1' else 'F2M'}"
          f"{' + gyro D455' if use_gyro else ' (visual murni)'}")
    print("  Encoder roda TIDAK dipakai untuk lokalisasi.")
    print("=" * 74)

    for pkg in ('rtabmap_odom', 'rtabmap_slam'):
        if not ros_pkg_exists(pkg):
            print(f"[ERROR] Paket ROS '{pkg}' tidak ada. Pasang: sudo apt install ros-$ROS_DISTRO-rtabmap-ros")
            sys.exit(1)

    webots_bin = shutil.which('webots')
    if not webots_bin:
        print("[ERROR] Binary 'webots' tidak ditemukan di PATH!")
        sys.exit(1)

    # 1. Webots
    webots_args = ['--minimize'] if os.environ.get('WEBOTS_MINIMIZE') == '1' else []
    driver_env = os.environ.copy()
    port = os.environ.get('WEBOTS_PORT')
    if port:
        webots_args.append(f'--port={port}')
        driver_env['WEBOTS_CONTROLLER_URL'] = f'ipc://{port}'
    print(f"[1/5] Menjalankan Webots: {WORLD_FILE} {' '.join(webots_args)}")
    p_webots = subprocess.Popen([webots_bin] + webots_args + [WORLD_FILE], start_new_session=True)
    procs.append(p_webots)
    time.sleep(3.5)

    # 2. Driver D455
    print("[2/5] Menjalankan driver D455 (d455_rtabmap_driver.py)...")
    p_driver = subprocess.Popen(['python3', DRIVER_NODE], start_new_session=True, env=driver_env)
    procs.append(p_driver)
    time.sleep(1.5)

    # 3. Odometri kamera (+ tebakan rotasi gyro D455)
    guess_args = []
    if use_gyro:
        print("[3/5] Menjalankan tebakan rotasi gyro D455 (d455_gyro_guess.py)...")
        p_gyro = subprocess.Popen(['python3', GYRO_NODE] + SIM_TIME, start_new_session=True)
        procs.append(p_gyro)
        guess_args = ['-p', 'guess_frame_id:=odom_gyro']
        time.sleep(0.5)
    print(f"[3/5] Menjalankan odometri kamera rtabmap_odom/rgbd_odometry ({'F2F' if strategy_id == '1' else 'F2M'})...")
    odom_cmd = [
        'ros2', 'run', 'rtabmap_odom', 'rgbd_odometry',
        '--ros-args',
        '--params-file', ODOM_PARAMS,
        '-p', f'Odom/Strategy:="{strategy_id}"',
    ] + guess_args + CAMERA_REMAP + [
        '-r', 'odom:=/odom',
        '-r', 'odom_info:=/odom_info',
        '-r', 'odom_last_frame:=/odom_last_frame',
        '-r', 'odom_local_map:=/odom_local_map',
    ]
    p_odom = subprocess.Popen(odom_cmd, start_new_session=True)
    procs.append(p_odom)
    time.sleep(1.0)

    # 4. RTAB-Map SLAM
    print("[4/5] Menjalankan RTAB-Map RGB-D + ICP SLAM...")
    rtabmap_cmd = [
        'ros2', 'run', 'rtabmap_slam', 'rtabmap',
        '--ros-args',
        '--params-file', RTAB_PARAMS,
        '-p', f'database_path:={DB_PATH}',
    ] + CAMERA_REMAP + [
        '-r', 'scan:=/scan',
        '-r', 'odom:=/odom',
        '-r', 'odom_info:=/odom_info',
        '-r', 'grid_map:=/map',
        '-r', 'cloud_map:=/rtabmap/cloud_map',
        '-r', 'mapPath:=/rtabmap/mapPath',
        '--', '--delete_db_on_start',
    ]
    p_slam = subprocess.Popen(rtabmap_cmd, start_new_session=True)
    procs.append(p_slam)

    # 5. RViz2
    rviz_bin = None if os.environ.get('NO_RVIZ') == '1' else shutil.which('rviz2')
    if rviz_bin:
        print(f"[5/5] Menjalankan RViz2: {RVIZ_CONFIG}")
        p_rviz = subprocess.Popen([rviz_bin, '-d', RVIZ_CONFIG] + SIM_TIME, start_new_session=True)
        procs.append(p_rviz)

    print("\n" + "=" * 74)
    print("  SELURUH LAYER AKTIF! Buka terminal baru untuk menggerakkan robot:")
    print(f"  python3 {os.path.join(BASE_DIR, 'src', 'teleop_explorer.py')}")
    print("  Atau jalan otomatis memetakan ruangan (frontier / random):")
    print(f"  python3 {os.path.join(BASE_DIR, 'src', 'auto_explorer.py')} [--mode random]")
    print("  Cek drift odometri kamera & SLAM:")
    print(f"  python3 {os.path.join(BASE_DIR, 'tools', 'cek_drift_slam.py')}")
    print("=" * 74 + "\n")

    while True:
        time.sleep(1.0)
        if p_webots.poll() is not None:
            print("[INFO] Webots ditutup. Menghentikan seluruh node...")
            cleanup()
        if p_odom.poll() is not None:
            print("[ERROR] rgbd_odometry berhenti. Menghentikan seluruh node...")
            cleanup()


if __name__ == '__main__':
    main()
