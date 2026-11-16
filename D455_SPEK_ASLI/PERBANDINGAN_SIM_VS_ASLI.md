# Perbandingan: Simulasi `BRONE_RESEARCH` vs D455 Asli

Tanggal       : 2026-11-09
Simulasi      : `/home/bromarku/brone/BRONE_RESEARCH` (file terakhir diubah 2026-11-03, belum disesuaikan)
Data kamera   : `SPESIFIKASI_D455.md` (serial 135122251358, diukur dalam kondisi terbalik & diam, via USB 2.1)

## Yang berbeda

| Aspek | Simulasi | Kamera asli | Dampak | Lokasi di kode |
|---|---|---|---|---|
| **FOV depth** | 86° (1.5010 rad) | **79.16°** | Besar: sudut `/scan` & proyeksi 3D meleset sampai ~3.4° di tepi | `worlds/openrmf_warehouse.wbt` (D455_DEPTH `fieldOfView`), `src/layer2_d455_scan.py:32` |
| **fx depth** | ~343 | **387.07** | Besar: point cloud/peta "melebar" | diturunkan dari FOV di `rtabmap_rgbd/src/d455_rtabmap_driver.py:92-98` |
| **FOV / fx color** | 86° / 343 | ~80.2° / 379.9 × 379.5 | Besar untuk VO & RTAB-Map | `.wbt` (D455_COLOR `fieldOfView`), fallback `rtabmap_rgbd/src/rgbd_visual_odometry.py:36-39` |
| Titik pusat color (cy) | 239.5 | **246.6** | Kecil (~1°) | `d455_rtabmap_driver.py:97-98` |
| Distorsi color | nol | k1 −0.059, k2 0.071, p1 −0.001, p2 −0.0002, k3 −0.023 | Kecil–sedang di tepi | `d455_rtabmap_driver.py:256` |
| Posisi color vs depth | di titik yang sama | **terpisah 59 mm** | Sedang: di kamera asli depth harus di-align ke color | `.wbt` (translation kedua sensor sama) |
| **Jarak minimum depth** | 0.05 m (world), 0.12–0.15 m (kode) | **0.293 m** | Besar: benda < 0.3 m terlihat di sim, hilang di kamera asli | `.wbt` `minRange 0.05`, `layer2_d455_scan.py:33,40`, `d455_rtabmap_driver.py:105,220`, `robot_driver_node.py:91`, `config/slam_params.yaml:25`, `rtabmap_params.yaml` (`Vis/MinDepth`, `Grid/RangeMin`), `rgbd_visual_odometry.py:77` |
| Rem anti-tabrak | aktif di 0.28 m | kamera tidak bisa melihat < 0.29 m | **Rem tidak akan pernah aktif di kamera asli** | `d455_rtabmap_driver.py:170`, `src/robot_driver_node.py` (`_apply_wheel_commands`) |
| Piksel depth kosong | 0 % | **~39 %** (pita kiri ±30 px, permukaan terlalu dekat, bayangan tepi) | Sedang | — (belum dimodelkan) |
| Noise depth | nol | 1 mm (<1 m), 11 mm (1–2 m), 26 mm (2–3 m), 41 mm (3–4 m), naik ∝ Z² | Sedang: sim terlalu optimistis | — (belum dimodelkan) |
| Outlier depth | tidak ada | nilai palsu sampai 65.5 m | Kecil (sudah dipotong batas 6 m) | `d455_rtabmap_driver.py:220` |
| **Orientasi IMU** | yaw absolut dari InertialUnit Webots | **tidak ada**, hanya gyro + accel | Besar: odometri & EKF dapat yaw sempurna | `layer1_odometry.py:87` (pakai `imu_yaw`), `d455_rtabmap_driver.py:280,288,308`, `robot_driver_node.py` (`rpy[2]`), `ekf_sensor_fusion.py:162` |
| `/imu/data` angular_velocity.z | diisi `wz` dari odometri roda | harus dari gyro | Sedang | `d455_rtabmap_driver.py:311` |
| Bias / noise gyro | nol | bias ≤ 0.11 °/s (drift ~3°/30 s), noise 0.0025 rad/s | Besar untuk yaw jangka panjang | — (belum dimodelkan) |
| Noise accel | nol | 0.008 m/s², ‖g‖ ~0.7 % kurang | Kecil | — |
| Frame IMU | `camera_link` | optical frame, 30 mm dari depth | Kecil | `d455_rtabmap_driver.py:307` |
| Rate depth / color | ~31 Hz | 11.6 Hz di USB 2 (30 Hz di USB 3) | Tergantung port USB | `d455_rtabmap_driver.py:190` |
| Rate IMU | ~62.5 Hz (tiap step 16 ms) | 22 / 12 Hz di USB 2 (200 / 100 Hz di USB 3) | Tergantung port USB | `basicTimeStep 16` di `.wbt` |
| Orientasi pasang | tegak | saat ini terbalik (171.6°) | Perlu roll 180° di TF jika dipasang terbalik di robot | `d455_rtabmap_driver.py:142-166` |

## Yang sudah sama

- Resolusi 640×480 untuk color dan depth.
- Jarak maksimum efektif ~6 m.
- Frame optik sesuai REP-103, depth dalam meter.

## Kesimpulan

Tiga perbedaan paling penting:
1. **FOV & fx**: 86° di simulasi vs 79.16° asli.
2. **Jarak minimum depth**: 0.05 m vs 0.293 m, sehingga rem 0.28 m tidak berguna di kamera asli.
3. **IMU simulasi memberi yaw absolut**, padahal D455 asli hanya punya gyro yang ber-bias.

Akibatnya, hasil SLAM di simulasi saat ini kemungkinan **lebih bagus daripada yang didapat dengan kamera asli**.

## Catatan

- Rate stream & IMU diukur di USB 2.1; ukur ulang setelah pindah ke port dan kabel USB 3.
- Nilai fx color sedikit berubah antar sesi (379.597 vs 379.907), jadi baca dari `camera_info` saat runtime.
