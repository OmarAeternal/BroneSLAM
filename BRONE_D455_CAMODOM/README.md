# BRONE_D455_CAMODOM: SLAM dengan ODOMETRI KAMERA

Salinan `BRONE_D455_REAL` (folder lama **tidak diubah**). Robot, world, model sensor D455,
RTAB-Map SLAM, explorer, dan tools sama persis. Yang berbeda hanya **sumber odometri**:

| | BRONE_D455_REAL | BRONE_D455_CAMODOM (folder ini) |
|---|---|---|
| Odometri | encoder roda + gyro + VO LK (python) → EKF custom | **RGB-D D455 → `rtabmap_odom/rgbd_odometry`** (Frame-to-Map + bundle adjustment) |
| Rotasi antar-frame | gyro di EKF | gyro **internal D455** sebagai tebakan rotasi (`guess_frame_id`) |
| Encoder roda | dipakai | **tidak dipakai** (driver tetap publish `/wheel/odom`, tapi diabaikan) |
| Kovariansi `/odom` | dari EKF (yaw terlalu kecil) | dari registrasi visual |
| SLAM | RTAB-Map VisIcp | sama, + `subscribe_odom_info` |

Seluruh lokalisasi hanya memakai **satu sensor: kamera D455** (RGB + depth + gyro internalnya).

---

## Mulai cepat

```bash
# Terminal 1
cd ~/brone/BRONE_D455_CAMODOM
python3 rtabmap_rgbd/launch/run_camodom_suite.py      # tunggu "SELURUH LAYER AKTIF!"

# Terminal 2: pilih SALAH SATU
python3 src/teleop_explorer.py
python3 src/auto_explorer.py
python3 src/auto_explorer.py --2c

# Terminal 3 (opsional): ukur drift vs ground truth
python3 tools/cek_drift_slam.py --duration 600
```

**Penting:** biarkan robot **diam ±2 detik** setelah launcher siap, karena bias gyro D455
dikalibrasi di awal. Kalau robot bergerak saat itu, kalibrasi ditolak dan bias dianggap 0
(log: `Kalibrasi ditolak`).

Launcher lama (EKF) masih ada di folder ini sebagai pembanding:
`python3 rtabmap_rgbd/launch/run_rtabmap_suite.py`.

### Opsi (environment variable)

| Variabel | Nilai | Arti |
|---|---|---|
| `CAMODOM_GYRO` | `1` (default) / `0` | gyro D455 sebagai tebakan rotasi / visual murni |
| `CAMODOM_STRATEGY` | `f2m` (default) / `f2f` | Frame-to-Map / Frame-to-Frame |
| `D455_PROFILE` | `usb3` / `usb2` | laju data kamera & IMU (sama seperti folder lama) |
| `D455_REALISM` | `1` / `0` | noise & artefak kamera asli ON / OFF |
| `D455_BRAKE_DIST` | `0.40` | rem anti-tabrak (m) |
| `NO_RVIZ`, `WEBOTS_MINIMIZE` | `1` | tanpa RViz / Webots diperkecil |
| `WEBOTS_PORT` | mis. `1240` | jalan bersamaan dengan simulasi lain (pakai juga `ROS_DOMAIN_ID` berbeda) |

RTAB-Map memakai database `~/.ros/rtabmap_camodom.db`, terpisah dari `~/.ros/rtabmap.db`
milik folder lama, jadi keduanya tidak saling hapus.

---

## Arsitektur

```
Gyro D455 ──► d455_gyro_guess.py ──► TF odom_gyro → base_link   (yaw = ∫gyro, translasi 0)
                                              │ (tebakan antar-frame)
RGB + Depth D455 ──► rgbd_odometry (F2M) ──► TF odom → odom_gyro   (koreksi visual)
                         │                    /odom (+kovariansi), /odom_info
                         ▼
RGB + Depth + /scan + /odom + /odom_info ──► RTAB-Map ──► /map + TF map → odom
```

TF akhir: `map → odom → odom_gyro → base_link`. Explorer dan tools tetap membaca
`map→base_link` dan `odom→base_link`, jadi tidak ada yang perlu diubah di sana.

### File baru (dibanding BRONE_D455_REAL)

| File | Isi |
|---|---|
| `rtabmap_rgbd/launch/run_camodom_suite.py` | launcher: Webots → driver → gyro guess → rgbd_odometry → RTAB-Map → RViz |
| `rtabmap_rgbd/config/camera_odom_params.yaml` | parameter `rgbd_odometry` (F2M, optical flow, depth 0.3–5 m, Force3DoF) |
| `rtabmap_rgbd/config/rtabmap_params_camodom.yaml` | parameter SLAM = versi lama + `subscribe_odom_info: true` |
| `rtabmap_rgbd/src/d455_gyro_guess.py` | integrasi gyro D455 → TF tebakan rotasi (kalibrasi bias otomatis, sumbu IMU diputar lewat TF) |

`ekf_sensor_fusion.py` dan `rgbd_visual_odometry.py` tidak dipakai launcher baru.

---

## Kenapa memakai gyro D455?

Uji awal dengan **visual murni** (feature matching, tanpa gyro): odometri hilang 23 kali dalam
60 detik pertama, terutama saat robot berputar 360°. Dinding apartemen di world ini polos,
sehingga kadang hanya ada 0–25 match. Robot terjebak di fase putar karena yaw-nya tidak
bertambah, dan error-nya 1.3 m setelah jalan 0.9 m.

Dengan gyro D455 sebagai tebakan + optical flow, rotasi tetap terlacak saat VO kehilangan fitur,
dan drift gyro langsung dikoreksi lagi setiap kali VO pulih.

## Hasil uji

RESULTS_PLACEHOLDER

## Batasan

- Saat VO hilang, **translasi tidak terukur** (tebakan gyro hanya rotasi). Kalau robot sedang
  maju di depan dinding polos, jarak itu hilang dari odometri. RTAB-Map mengoreksinya lewat loop closure.
- Kamera buta < 0.293 m: terlalu dekat dinding berarti tidak ada fitur.
- D455 asli: IMU 200 Hz (sim 62.5 Hz) dan frame IMU berorientasi optik. `d455_gyro_guess.py`
  memutar sumbu memakai TF dari `header.frame_id` IMU ke `base_link`, jadi TF itu harus ada
  (otomatis dari `realsense2_camera` + TF `base_link→camera_link` robot). Di driver
  `realsense2_camera`, aktifkan `enable_gyro:=true unite_imu_method:=2`.
