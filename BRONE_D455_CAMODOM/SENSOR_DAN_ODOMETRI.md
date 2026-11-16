# Sensor, Lokalisasi & Opsi Odometri — BRONE_D455_REAL

Dokumen ini merangkum sensor apa saja yang dibaca oleh stack RTAB-Map
(`rtabmap_rgbd/launch/run_rtabmap_suite.py`) untuk mapping dan lokalisasi, ditambah
opsi pengganti/pelengkap EKF untuk odometri. Tidak ada kode yang diubah.

---

## 1. Sensor yang dibaca (`rtabmap_rgbd/src/d455_rtabmap_driver.py`)

| Sensor | Device Webots | Topik ROS 2 | Dipakai oleh |
|---|---|---|---|
| **D455 Color (RGB)** | `d455_color` (640×480, FOV 80.2°) | `/camera/color/image_raw`, `/camera/color/camera_info` | VO, RTAB-Map |
| **D455 Depth** | `d455_depth` (640×480, FOV 79.16°, min 0.293 m) | `/camera/aligned_depth_to_color/image_raw` (align ke color), `/camera/depth/image_rect_raw` (mentah) | VO, RTAB-Map |
| **Scan semu dari depth** | diturunkan dari depth mentah (bukan sensor sendiri) | `/scan` (LaserScan, 160 beam, frame `camera_link`) | RTAB-Map (ICP), peta, rem anti-tabrak, auto_explorer |
| **Gyro (IMU D455)** | `gyro` + bias & noise hasil uji | `/imu/data` → `angular_velocity` | EKF (hanya `wz`), yaw odometri roda |
| **Accelerometer (IMU D455)** | `accelerometer` + noise | `/imu/data` → `linear_acceleration` | dipublish, **tidak dipakai** EKF |
| **Encoder roda ×3** | `wheel1_sensor`..`wheel3_sensor` | `/wheel/odom` (vx, vy, wz) | EKF |
| InertialUnit | `imu` (yaw absolut) | – | **hanya ground truth**, tidak dipakai lokalisasi |
| GPS | `gps` | `/ground_truth/pose` | **hanya evaluasi** di RViz |

Input lain: `/cmd_vel` (perintah gerak dari teleop / auto_explorer), `/clock` (waktu simulasi).

**Inti:** perangkat keras yang benar-benar dipakai hanya **kamera D455 (RGB + Depth + IMU)**
dan **encoder 3 roda omni**. Tidak ada LiDAR; `/scan` adalah irisan horizontal dari depth.

---

## 2. Alur data mapping & lokalisasi

```
Encoder roda ──► /wheel/odom ─┐
Gyro D455 ─────► /imu/data ───┼──► EKF (ekf_sensor_fusion.py) ──► /odom + TF odom→base_link
RGB + Depth ──► VO (LK optical flow, rgbd_visual_odometry.py) ──► /vo/odom ─┘
                                                     │
RGB + Depth(aligned) + /scan + /odom ──► RTAB-Map ──► /map, /rtabmap/cloud_map + TF map→odom
```

- **Odometri lokal (EKF, 6 state: x, y, θ, vx, vy, wz)** menggabungkan:
  1. vx, vy dari encoder roda (`/wheel/odom`)
  2. wz dari gyro (`/imu/data`) — tanpa yaw absolut (D455 tidak punya orientasi)
  3. vx, vy dari Visual Odometry (`/vo/odom`): `goodFeaturesToTrack` → `calcOpticalFlowPyrLK` → back-projection 3D pakai depth
- **SLAM global (RTAB-Map, `config/rtabmap_params.yaml`)**:
  - `Reg/Strategy=2` (VisIcp): fitur visual RGB-D + PnP, dihaluskan ICP pada `/scan`
  - `Reg/Force3DoF=true` (robot darat, x-y-yaw)
  - `/odom` dari EKF sebagai tebakan awal
  - Loop closure dari pengenalan tempat visual (+ `RGBD/ProximityBySpace`)
  - Occupancy grid `/map` dari depth, rentang 0.3–5.5 m, sel 5 cm

### Masalah odometri yang sudah tercatat (lihat `CARA_MENJALANKAN.md` §5)

- Roda omni selip ~19 % (roda hanya menghitung ~81 % jarak asli).
- VO hanya menghitung ~69 % jarak (usb3, realisme ON) karena filter outlier di sekitar nol.
- Yaw hanya dari integrasi gyro → drift.
- Kovariansi yaw EKF terlalu kecil → RTAB-Map menolak sebagian loop closure.
- Accelerometer belum dipakai sama sekali.

---

## 3. Opsi odometri selain EKF buatan sendiri

### A. Ganti / perbaiki filter fusi (sensor tetap: roda + gyro + VO)

1. **`robot_localization` – `ekf_node`** (paket ROS standar)
   EKF 15-state (3D pose, kecepatan, akselerasi) yang sudah matang. Konfigurasi per sensor
   lewat matriks boolean (misalnya roda: vx, vy; IMU: vyaw, ax, ay; VO: vx, vy). Ada
   `two_d_mode` untuk robot darat. Bisa langsung mengganti `ekf_sensor_fusion.py`.
2. **`robot_localization` – `ukf_node`** (Unscented Kalman Filter)
   Input dan konfigurasinya sama dengan ekf_node. Lebih akurat bila model sangat nonlinier,
   tetapi sedikit lebih berat dan punya parameter tambahan (α, β, κ).
3. **Complementary filter**
   Rumusnya sederhana: `v = α·v_roda + (1−α)·v_VO` dan `θ = θ + wz_gyro·dt`. Sangat ringan,
   tetapi tidak ada kovariansi yang berarti untuk RTAB-Map.
4. **Factor graph / smoother** (GTSAM iSAM2, paket `fuse`, atau `Odom/Strategy` di RTAB-Map)
   Mengoptimasi jendela beberapa pose sekaligus, bukan hanya state terakhir. Lebih tahan
   terhadap data yang telat atau tidak urut (bug EKF yang dulu ditemukan). Implementasinya
   paling kompleks.

### B. Ganti sumber odometri (pakai node siap pakai)

5. **RTAB-Map `rgbd_odometry`**
   VO bawaan RTAB-Map dengan strategi Frame-to-Map (F2M, default) atau Frame-to-Frame.
   Memakai fitur GFTT/ORB + PnP RANSAC + bundle adjustment lokal. Bisa diberi `/imu/data`
   (`Odom/...` + `wait_imu_to_init`) dan tebakan dari roda (`guess_frame_id`). Hampir pasti
   lebih akurat daripada VO LK custom yang sekarang. Bisa langsung mengganti
   `rgbd_visual_odometry.py`, atau sekaligus menggantikan EKF.
6. **RTAB-Map `icp_odometry`** pada `/scan` atau point cloud depth
   Scan-matching geometris. Kuat di ruangan berdinding, tetapi lemah di lorong panjang yang
   seragam. FOV scan dari D455 hanya ~79°, jadi rawan degenerasi.
7. **Visual-Inertial Odometry (VIO)**: OpenVINS, VINS-Fusion (RGB-D/stereo + IMU),
   ORB-SLAM3 (RGB-D-Inertial)
   Fusi kamera dan IMU yang ketat (tightly coupled), sehingga accel ikut dipakai dan skala
   dapat diamati. Butuh kalibrasi kamera–IMU dan IMU yang cukup cepat. Di simulasi IMU
   terbatas 62.5 Hz, dan di profil usb2 hanya 21.6 Hz, padahal VIO idealnya ≥ 200 Hz.
8. **Wheel + gyro saja (dead reckoning)**
   Inilah `/wheel/odom` yang sekarang, tanpa EKF. Paling sederhana dan stabil dalam jangka
   pendek, tetapi selip 19 % langsung menjadi error jarak.

---

## 4. Tabel perbandingan

Skala: ★ rendah … ★★★ tinggi. "Effort" = usaha integrasi ke repo ini.

| Opsi | Sensor dipakai | Akurasi jangka pendek | Tahan drift / selip | Tahan scene minim tekstur | Beban CPU | Effort | Siap pakai ROS 2 | Catatan untuk BRONE |
|---|---|---|---|---|---|---|---|---|
| **EKF custom (sekarang)** | roda, gyro, VO | ★★ | ★★ | ★★ | ★ | – (sudah ada) | ✗ (kode sendiri) | Kovariansi yaw terlalu kecil, accel tak dipakai |
| **robot_localization ekf_node** | roda, gyro, accel, VO | ★★ | ★★ | ★★ | ★ | ★ | ✓ | Pengganti langsung; lebih teruji, `two_d_mode` |
| **robot_localization ukf_node** | sama | ★★–★★★ | ★★ | ★★ | ★★ | ★ | ✓ | Untung kecil dibanding EKF untuk robot 2D |
| **Complementary filter** | roda, gyro, VO | ★★ | ★ | ★★ | ★ | ★ | ✗ | Cocok untuk baseline / perbandingan skripsi |
| **Factor graph (GTSAM / fuse)** | semua | ★★★ | ★★★ | ★★ | ★★★ | ★★★ | sebagian (`fuse`) | Tahan data telat/tak urut; paling kompleks |
| **RTAB-Map rgbd_odometry (F2M)** | RGB, depth (+IMU, +roda sebagai guess) | ★★★ | ★★ | ★ | ★★ | ★ | ✓ | Kandidat terkuat untuk mengganti VO LK (masalah 69 %) |
| **RTAB-Map icp_odometry** | /scan atau depth cloud | ★★ | ★★ | ★★★ | ★★ | ★ | ✓ | FOV sempit 79° dan min 0.293 m → rawan di lorong |
| **VIO (OpenVINS / VINS-Fusion / ORB-SLAM3)** | RGB/depth + gyro + accel | ★★★ | ★★★ | ★★ | ★★★ | ★★★ | sebagian | Butuh IMU cepat dan kalibrasi; terbatas IMU 62.5 Hz di sim |
| **Roda + gyro saja** | encoder, gyro | ★★ | ★ | ★★★ | ★ | – (sudah ada) | – | Selip 19 % tidak terkoreksi |

### Rekomendasi singkat

1. **Paling murah dan efektif:** ganti `rgbd_visual_odometry.py` dengan **RTAB-Map
   `rgbd_odometry`** (beri `guess_frame_id` dari roda), lalu fusikan dengan roda + gyro
   lewat **`robot_localization` ekf_node**. Kovariansi yang realistis membantu RTAB-Map
   menerima loop closure.
2. **Kalau tetap memakai EKF custom:** naikkan kovariansi yaw (drift gyro), perbaiki filter
   outlier VO (di sekitar median, bukan nol), dan pertimbangkan memakai accel.
3. **Untuk riset lanjutan / robot asli dengan IMU 200 Hz:** coba VIO (OpenVINS atau
   ORB-SLAM3 RGB-D-Inertial) sebagai pembanding.
