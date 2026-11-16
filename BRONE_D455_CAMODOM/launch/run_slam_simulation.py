#!/usr/bin/env python3
"""
Master Runner: Simulasi Graph-Based SLAM (Layer 0 -> Layer 1 -> Layer 2 -> Layer 3)
Cara Pakai:
  1. Jalankan seluruh suite (Webots + Driver D455 + Graph-SLAM + RViz2):
     python3 launch/run_slam_simulation.py

  2. Di terminal terpisah, jalankan kontrol manual / Reactive Auto-Explore:
     python3 src/teleop_explorer.py
"""

import os
import sys
import time
import signal
import shutil
import argparse
import subprocess


def main():
    parser = argparse.ArgumentParser(description='Launch Full Bottom-Up D455 SLAM Simulation')
    parser.add_argument(
        '--slam-toolbox',
        action='store_true',
        help='Gunakan paket binary slam_toolbox (default: gunakan modul transparan src/layer3_graph_slam.py)'
    )
    parser.add_argument(
        '--no-webots',
        action='store_true',
        help='Jangan buka Webots baru jika Anda sudah membuka worlds/openrmf_warehouse.wbt secara manual'
    )
    parser.add_argument(
        '--auto-patrol',
        action='store_true',
        help='Langsung jalankan robot menjelajahi ruangan secara otomatis'
    )
    args = parser.parse_args()

    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    world_file = os.path.join(base_dir, 'worlds', 'openrmf_warehouse.wbt')
    driver_script = os.path.join(base_dir, 'src', 'robot_driver_node.py')
    layer3_script = os.path.join(base_dir, 'src', 'layer3_graph_slam.py')
    teleop_script = os.path.join(base_dir, 'src', 'teleop_explorer.py')
    slam_params = os.path.join(base_dir, 'config', 'slam_params.yaml')
    rviz_config = os.path.join(base_dir, 'config', 'slam_view.rviz')

    print("=" * 72)
    print("  LAUNCHING BRONE REALSENSE D455 SLAM SUITE (LAYER 0 -> LAYER 3)        ")
    print("=" * 72)
    print(f"  [Layer 0] World      : Multi-Room Apartment & Workspace (Zero-Download)")
    print(f"  [Layer 1] Odometri   : src/layer1_odometry.py (3 Encoder + D455 IMU)")
    print(f"  [Layer 2] Persepsi   : src/layer2_d455_scan.py (Pure D455 86° Depth)")
    print(f"  [Layer 3] Graph SLAM : {'slam_toolbox' if args.slam_toolbox else 'src/layer3_graph_slam.py'} + config/slam_params.yaml")
    print("=" * 72)

    processes = []

    try:
        # 1. Layer 0: Webots Simulator
        if not args.no_webots and shutil.which('webots'):
            print("[1/4] Membuka Layer 0: Webots (openrmf_warehouse.wbt)...")
            p_webots = subprocess.Popen(
                ['webots', '--mode=realtime', world_file],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            processes.append(('Webots Simulator', p_webots))
            time.sleep(2.5)

        # 2. Layer 1 & Layer 2: Robot Driver Node (Webots <-> ROS 2)
        print("[2/4] Menjalankan Layer 1 & Layer 2: Driver Robot + RealSense D455...")
        p_driver = subprocess.Popen([sys.executable, driver_script])
        processes.append(('Layer 1 & 2 Driver', p_driver))
        time.sleep(1.0)

        # 3. Layer 3: Graph-Based SLAM Node
        if args.slam_toolbox:
            print("[3/4] Menjalankan Layer 3: SLAM Toolbox (config/slam_params.yaml)...")
            p_slam = subprocess.Popen([
                'ros2', 'run', 'slam_toolbox', 'async_slam_toolbox_node',
                '--ros-args', '--params-file', slam_params
            ])
        else:
            print("[3/4] Menjalankan Layer 3: Graph-Based SLAM Node (src/layer3_graph_slam.py)...")
            p_slam = subprocess.Popen([sys.executable, layer3_script])
        processes.append(('Layer 3 Graph SLAM', p_slam))
        time.sleep(1.0)

        # 4. RViz2 Visualization
        if shutil.which('rviz2'):
            print("[4/4] Membuka Visualisasi RViz2 (config/slam_view.rviz)...")
            p_rviz = subprocess.Popen(
                ['rviz2', '-d', rviz_config],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            processes.append(('RViz2', p_rviz))

        if args.auto_patrol:
            print("[AUTO] Menjalankan Reactive Auto-Explore...")
            p_patrol = subprocess.Popen(
                [sys.executable, teleop_script],
                stdin=subprocess.DEVNULL
            )
            processes.append(('Auto-Patrol', p_patrol))

        print("\n[SIAP] Simulasi SLAM sedang berjalan!")
        print("-> Buka terminal baru dan jalankan: python3 src/teleop_explorer.py")
        print("-> Tekan Ctrl+C di sini kapan saja untuk menutup seluruh proses.\n")

        while True:
            time.sleep(1.0)
            # Auto-reconnect driver jika Webots baru selesai loading
            if p_driver.poll() is not None:
                print("[INFO] Menghubungkan ulang Driver ke Webots...")
                p_driver = subprocess.Popen([sys.executable, driver_script])
                processes[1] = ('Layer 1 & 2 Driver', p_driver)

    except KeyboardInterrupt:
        print("\n[INFO] Menghentikan seluruh proses simulasi...")
    finally:
        for name, proc in reversed(processes):
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    proc.kill()
        print("[SELESAI] Semua node telah dihentikan dengan bersih.")


if __name__ == '__main__':
    main()
