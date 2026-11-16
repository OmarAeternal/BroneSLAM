# Spesifikasi Asli Intel RealSense D455 (hasil ukur langsung)

Tanggal ukur : 2026-11-09
Kondisi      : kamera **terbalik (upside-down)** dan **diam**, menghadap kolong meja (± 0.3–3.8 m)
Driver       : librealsense + `realsense2_camera` (ROS 2 Jazzy)
Data mentah  : folder `raw/` (lihat daftar di bagian akhir)

---

## 1. Identitas perangkat

| Item | Nilai |
|---|---|
| Model | RealSense D455 (Product ID 0B5C, lini D400) |
| Serial Number | 135122251358 |
| ASIC Serial | 112523060459 |
| Firmware | 5.13.0.50 |
| IMU | Bosch BMI085 (gyro + accel, **tanpa magnetometer, tanpa orientasi absolut**) |
| Advanced Mode | YES |
| Koneksi | **USB 2.1** ← seharusnya USB 3.x (lihat bagian 7) |

## 2. Geometri stereo & susunan sensor

| Item | Nilai |
|---|---|
| Baseline stereo | **94.97 mm** |
| Depth units | 0.001 m (1 mm per nilai Z16) |
| Translasi Depth → Color | x = **−59.12 mm**, y = +0.15 mm, z = +0.58 mm (rotasi ≈ 0.3°) |
| Translasi Depth → IMU (gyro & accel) | x = −30.22 mm, y = +7.40 mm, z = +16.02 mm (rotasi identitas) |

Semua translasi di atas dalam frame optik depth (x kanan, y bawah, z depan).

## 3. Intrinsik kamera (kalibrasi pabrik dari perangkat ini)

### Depth (Z16), distorsi = nol
| Resolusi | fx = fy | cx | cy | FOV H × V |
|---|---|---|---|---|
| 1280×720 | 645.116 | 638.558 | 356.195 | **89.54° × 58.33°** |
| **640×480** | **387.070** | **319.135** | **237.717** | **79.16° × 63.60°** |
| 256×144 | 645.116 | 126.558 | 68.195 | — |

### Color (RGB), model distorsi Inverse Brown-Conrady
| Resolusi | fx | fy | cx | cy | FOV H × V |
|---|---|---|---|---|---|
| 1280×800 | 633.178 | 632.422 | 636.989 | 411.009 | **90.61° × 64.61°** |
| **640×480** | **379.907** | **379.453** | **318.194** | **246.605** | **≈ 80.2° × 64.6°** |

Koefisien distorsi color (k1, k2, p1, p2, k3):
`-0.058852, 0.071007, -0.001030, -0.000151, -0.022689`

Catatan: fx color 640×480 terbaca 379.597 pada sesi pertama hari ini dan 379.907 pada sesi
berikutnya (selisih 0.08 %). Pakai nilai dari `camera_info` saat runtime, jangan di-hardcode.

> Penting: FOV "86°/87°" di datasheet hanya berlaku untuk resolusi 16:9 penuh.
> Di 640×480 (4:3), FOV horizontal depth hanya **79.16°**.

## 4. Mode stream yang didukung

| Stream | Resolusi / FPS |
|---|---|
| Depth Z16 | 1280×720@5 · 848×480@10/5 · 640×480@30/15/5 · 640×360@30 · 480×270@60/30/15/5 · 256×144@90 |
| Color (RGB8/BGR8/RGBA8/BGRA8/YUYV) | 1280×800@8 · 1280×720@15/10/5 · 640×480@30/15/5 · 424×240@60/30/15/5 |
| Infrared (Y8/RGB) | 1280×720@5 · 848×480@10/5 · 640×480@30/15/5 · 640×360@30 · 480×270@60/30/15/5 |
| Accel | 400/200/100 Hz |
| Gyro | 400/200 Hz |

Daftar ini **dibatasi oleh USB 2**. Di USB 3, D455 mendukung depth sampai 1280×720@30 dan
848×480@90.

Opsi default stereo: Laser Power 150 (0–360), Emitter ON, Auto Exposure ON (33 ms), Gain 16.

## 5. Hasil ukur depth (diam, 640×480 diminta @15 FPS, 20 detik)

| Item | Nilai |
|---|---|
| FPS aktual depth / color | **11.6 / 12.0 FPS** (diminta 15) |
| Jeda frame terlama | 667 ms (depth), 267 ms (color) |
| Piksel valid rata-rata | **61.3 %** |
| Piksel yang selalu valid | 56.9 % |
| Jarak terdekat terbaca | **0.293 m** (benda yang lebih dekat → 0 / hilang) |
| Persentil 1 % / median / 99 % | 0.556 m / 2.794 m / 3.819 m |
| Nilai maks (outlier palsu) | 65.535 m (= nilai Z16 maksimum, harus difilter) |

Noise temporal (std per piksel, kamera diam):

| Rentang jarak | Std | Error relatif |
|---|---|---|
| 0.3 – 1 m | 1.0 mm | 0.16 % |
| 1 – 2 m | 10.8 mm | 0.59 % |
| 2 – 3 m | 26.4 mm | 1.01 % |
| 3 – 4 m | 41.0 mm | 1.23 % |

Noise naik kira-kira sebanding dengan Z² (sesuai teori stereo).

Pola piksel kosong yang terlihat (`raw/depth_median.png`):
- pita kosong ±30 px di sisi **kiri** gambar (daerah yang tidak terlihat oleh kedua imager),
- permukaan yang terlalu dekat (< 0.3 m, di sini kolong meja) seluruhnya kosong,
- bayangan stereo di tepi benda depan.

## 6. Hasil ukur IMU (diam, terbalik, 30 detik)

| Item | Gyro | Accel |
|---|---|---|
| Frame ROS | `camera_gyro_optical_frame` | `camera_accel_optical_frame` |
| Rate diminta | 200 Hz | 100 Hz |
| **Rate aktual** | **21.6 Hz** (jeda maks 670 ms) | **12.2 Hz** |
| Bias rata-rata (x, y, z) | −0.00185, +0.00089, −0.00034 rad/s (−0.106, +0.051, −0.020 °/s) | — |
| Noise std (x, y, z) | 0.0025, 0.0027, 0.0015 rad/s | 0.0076, 0.0083, 0.0087 m/s² |
| Rata-rata (x, y, z) | — | +0.017, **+9.605**, +1.416 m/s² |
| ‖g‖ terukur | — | 9.709 m/s² (≈ 0.7 % di bawah g lokal ~9.78) |

Drift kalau bias gyro tidak dikoreksi: ±3.2° per 30 detik (sumbu x).

**Verifikasi posisi terbalik:** vektor "atas" dari accelerometer = (0.002, **+0.989**, 0.146)
dalam frame optik. Kalau kamera tegak, "atas" = −Y optik. Sudut terhadap posisi tegak
= **171.6°**, artinya kamera terbalik dan menengadah sekitar 8.4°. Ini cocok dengan gambar
color (`raw/color.png`) yang memperlihatkan kolong meja di bagian atas.
→ Kalau dipasang terbalik di robot, perlu rotasi 180° di sumbu optik (roll π) pada TF
`camera_link`, atau set `rotation` di driver.

Kalibrasi IMU di perangkat: sensitivitas accel ≈ 1.007 (offset 0.033, 0.123, 0.230 m/s²),
varians bias/noise di tabel pabrik = 0 (tidak terisi).

## 7. Masalah koneksi yang tercatat (USB 2.1)

- Color + depth + IMU sekaligus: frame tidak masuk ("Frame Corrupted", "Frames didn't arrive within 5 seconds").
- Kamera sempat reset sendiri (nomor device USB berubah 005 → 006).
- FPS depth/color hanya ~11.6 dari 15; IMU hanya ~22/12 Hz dari 200/100 Hz.
- Saran: pindah ke port USB 3 (biru/SS) dan kabel USB-C yang mendukung USB 3, lalu ukur ulang
  bagian 5 dan 6.

## 8. Isi folder `raw/`

| File | Isi |
|---|---|
| `enumerate.txt` | info perangkat + mode stream (`rs-enumerate-devices`) |
| `calibrasi.txt` | semua intrinsik/ekstrinsik semua resolusi (`rs-enumerate-devices -c`) |
| `options.txt` | semua opsi sensor + rentang/default (`rs-enumerate-devices -o`) |
| `depth_static.json` | statistik depth & camera_info ROS |
| `imu_static.json` | statistik IMU diam |
| `color.png`, `depth_median.png` | snapshot scene saat pengukuran |
| `node_depth.log`, `node_imu.log` | log driver ROS saat pengukuran |
