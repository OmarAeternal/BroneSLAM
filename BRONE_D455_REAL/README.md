# BRONE_D455_REAL

Simulasi SLAM (Webots + ROS 2 Jazzy) untuk robot **BRONE**: robot 3 roda omni dengan satu
kamera **Intel RealSense D455** (RGB + depth + IMU) dan encoder roda, tanpa LiDAR.
Model sensornya disesuaikan dengan D455 asli (serial 135122251358) dari data uji 2026-11-09
dan datasheet: FOV, jarak minimum 0.293 m, noise depth, bias dan noise gyro, serta laju data USB.

Folder ini adalah salinan `BRONE_RESEARCH`. Folder lama itu tidak diubah.

---

## Mulai cepat

```bash
# Terminal 1: Webots + driver D455 + VO + EKF + RTAB-Map + RViz2
cd ~/brone/BRONE_D455_REAL
python3 rtabmap_rgbd/launch/run_rtabmap_suite.py      # tunggu "SELURUH LAYER AKTIF!"

# Terminal 2: pilih SALAH SATU
python3 src/teleop_explorer.py                        # manual: W/S A/D Q/E, P = auto, X = keluar
python3 src/auto_explorer.py                          # otomatis: frontier, petakan seluruh ruangan
python3 src/auto_explorer.py --mode random            # otomatis: jalan acak
```

Opsi environment variable: `D455_PROFILE=usb3|usb2`, `D455_REALISM=1|0`,
`D455_BRAKE_DIST=0.40`, `NO_RVIZ=1`, `WEBOTS_MINIMIZE=1`.
Penjelasan lengkap ada di [CARA_MENJALANKAN.md](CARA_MENJALANKAN.md).

---

## Arsitektur

```
Encoder roda ──► /wheel/odom ─┐
Gyro D455 ─────► /imu/data ───┼──► EKF ──► /odom + TF odom→base_link
RGB + Depth ──► Visual Odometry ──► /vo/odom ─┘
                                        │
RGB + Depth + /scan (irisan depth) + /odom ──► RTAB-Map ──► /map + TF map→odom
```

| Sensor | Topik | Dipakai untuk |
|---|---|---|
| D455 color | `/camera/color/image_raw`, `/camera/color/camera_info` | VO, RTAB-Map |
| D455 depth (di-align ke color) | `/camera/aligned_depth_to_color/image_raw` | VO, RTAB-Map, peta |
| Scan semu dari depth | `/scan` | ICP RTAB-Map, rem anti-tabrak, explorer |
| IMU D455 (gyro + accel) | `/imu/data` | EKF (yaw-rate) |
| Encoder 3 roda | `/wheel/odom` | EKF |
| GPS / InertialUnit (sim) | `/ground_truth/pose` | hanya evaluasi |

Rincian sensor dan opsi odometri ada di [SENSOR_DAN_ODOMETRI.md](SENSOR_DAN_ODOMETRI.md).

---

## Struktur folder

```
BRONE_D455_REAL/
├── rtabmap_rgbd/                  # Suite utama (model D455 asli)
│   ├── launch/run_rtabmap_suite.py    launcher semua layer
│   ├── src/d455_rtabmap_driver.py     bridge Webots → ROS 2 (kamera, IMU, encoder, motor, /scan)
│   ├── src/d455_real_model.py         semua angka model D455 asli (noise, FOV, bias, laju)
│   ├── src/rgbd_visual_odometry.py    VO: optical flow LK + back-projection depth → /vo/odom
│   ├── src/ekf_sensor_fusion.py       EKF 6-state: roda + gyro + VO → /odom
│   └── config/                        rtabmap_params.yaml, rtabmap_view.rviz
├── src/
│   ├── layer1_odometry.py             kinematika & odometri roda omni
│   ├── layer2_d455_scan.py            depth → LaserScan + deteksi halangan depan
│   ├── layer3_graph_slam.py           graph-SLAM 2D bawaan (fallback bila rtabmap tidak ada)
│   ├── robot_driver_node.py           driver untuk pipeline 2D lama
│   ├── teleop_explorer.py             teleop keyboard + auto-explore reaktif
│   ├── auto_explorer.py               explorer frontier/random berbasis peta + lokalisasi
│   ├── explore_core.py                inti explorer (tanpa ROS, bisa diuji offline)
│   └── room_grid.py                   grid ruangan 8×8 (navigasi "pergi ke kotak")
├── launch/run_slam_simulation.py  # pipeline 2D lama (belum memakai model noise/IMU)
├── config/                        # parameter & RViz untuk pipeline 2D lama
├── tools/
│   ├── cek_sensor_d455.py             bandingkan sensor simulasi vs kamera asli
│   └── buat_peta_grid.py              gambar grid ruangan di atas foto Webots
└── worlds/openrmf_warehouse.wbt   # world Webots
```

---

## Dokumentasi

| File | Isi |
|---|---|
| [CARA_MENJALANKAN.md](CARA_MENJALANKAN.md) | Cara menjalankan, opsi, hasil cek sensor, daftar perubahan dari sim lama, temuan |
| [SENSOR_DAN_ODOMETRI.md](SENSOR_DAN_ODOMETRI.md) | Sensor yang dibaca, alur lokalisasi/mapping, opsi odometri selain EKF + tabel perbandingan |
| [IMPLEMENTASI_JETSON_D455.pdf](IMPLEMENTASI_JETSON_D455.pdf) | Langkah memindahkan SLAM ini ke Jetson + D455 asli |

---

## Kebutuhan

- Ubuntu 24.04 + ROS 2 Jazzy
- Webots (binary `webots` ada di PATH)
- `ros-jazzy-rtabmap-ros` (opsional; tanpa paket ini launcher memakai `layer3_graph_slam.py`)
- Python: `numpy`, `opencv-python` (`cv2`)

## Batasan yang diketahui

- Distorsi lensa dan titik pusat asli color belum dimodelkan (citra setara sudah di-rectify).
- IMU dibatasi 62.5 Hz oleh langkah Webots 16 ms (D455 asli 200 Hz).
- VO menghitung jarak terlalu pendek (~69 %) dengan noise depth asli; roda omni selip ~19 %.
- Detailnya ada di bagian 4–5 [CARA_MENJALANKAN.md](CARA_MENJALANKAN.md).
