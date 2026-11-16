# BRONE_D455_REAL: Simulasi SLAM dengan sensor D455 spek asli

Folder ini adalah salinan `BRONE_RESEARCH` (yang lama **tidak diubah**). Sensor kameranya
disesuaikan dengan D455 asli (serial 135122251358) memakai data uji 2026-11-09
(`~/brone/D455_SPEK_ASLI/`) ditambah datasheet.

## 1. Cara menjalankan

Buka terminal 1 (ROS 2 Jazzy sudah ter-source):

```bash
cd ~/brone/BRONE_D455_REAL
python3 rtabmap_rgbd/launch/run_rtabmap_suite.py
```

Launcher ini menjalankan Webots, driver D455, Visual Odometry, EKF, RTAB-Map, dan RViz2.
Tunggu sampai muncul tulisan `SELURUH LAYER AKTIF!`.

Buka terminal 2 untuk menggerakkan robot:

```bash
python3 ~/brone/BRONE_D455_REAL/src/teleop_explorer.py
```

Tombol: W/S maju-mundur, A/D geser, Q/E putar, SPASI berhenti, **P = auto-explore**, X keluar.

Untuk berhenti, tekan Ctrl+C di terminal 1 (atau tutup Webots).

### Jalan otomatis memetakan ruangan (`auto_explorer.py`)

Pakai **salah satu** saja: teleop **atau** auto-explorer (keduanya mengirim `/cmd_vel`).

```bash
python3 ~/brone/BRONE_D455_REAL/src/auto_explorer.py                 # FRONTIER: petakan seluruh ruangan
python3 ~/brone/BRONE_D455_REAL/src/auto_explorer.py --mode random   # RANDOM: jalan acak terus-menerus
```

- **frontier**: putar 360° di awal, lalu bergerak ke batas area yang belum terpetakan.
  Setiap kali sampai, robot putar 360° lagi. Proses diulang sampai tidak ada frontier,
  kemudian robot kembali ke titik awal dan berhenti dengan pesan `SELESAI`.
- **random**: memilih titik acak (0.8–4 m) di area yang sudah terpetakan, berjalan ke sana, lalu
  memilih titik berikutnya. Mode ini tetap memakai peta dan lokalisasi, jadi tidak menabrak.
- **Anti-tabrak berbasis lokalisasi**: pose diambil dari TF `map→base_link` (RTAB-Map + EKF). Jarak
  **tepi** robot (radius 0.13 m) ke tembok dihitung dari `/map` ditambah memori `/scan` 5 detik
  terakhir, karena D455 buta di bawah 0.293 m. Setiap perintah diprediksi 1 detik ke depan dan
  ditolak atau diperlambat kalau jaraknya akan kurang dari **5 cm**. Perencana jalur menambah
  cadangan 3 cm untuk error lokalisasi dan lebih memilih lewat tengah lorong (±30 cm dari tembok).
  Kalau TF atau peta hilang, robot berhenti. Ctrl+C mengirim STOP.
- Opsi: `--min-clearance 0.05`, `--pose-margin 0.03`, `--max-speed 0.22`, `--duration 600`,
  `--no-spin`, `--no-return`, `--seed 1`.
- RViz: tambahkan topik `/explore/path` (Path) dan `/explore/goal` (Pose).

#### Pergi ke kotak grid (mode goto)

Ruangan dibagi menjadi grid 8×8. Lihat gambar **`peta_grid/peta_grid.png`**:
kolom **a–h** dari kiri ke kanan, baris **1–8** dari atas ke bawah, satu kotak 1.55 × 1.64 m.
Robot mulai di kotak **3E**.

```bash
python3 ~/brone/BRONE_D455_REAL/src/auto_explorer.py --2c         # pergi ke 2C lalu berhenti
python3 ~/brone/BRONE_D455_REAL/src/auto_explorer.py --2c --8h    # ke 2C, lalu ke 8H
python3 ~/brone/BRONE_D455_REAL/src/auto_explorer.py --grid       # tampilkan grid di terminal
```

- Penulisan `--2c`, `--C2`, dan `--goto 2c,8h` sama artinya.
- Robot bisa langsung menuju kotak yang belum terpetakan. Jalur lewat area kosong boleh
  direncanakan, dan karena robot selalu menghadap arah gerak, jalurnya diperbarui tiap 2 detik
  saat kamera melihat tembok baru.
- Robot berhenti di titik aman di dalam kotak yang paling dekat dengan pusat kotak. Kalau pusat
  kotak tertutup perabot (misalnya kasur), robot berhenti di pinggirnya.
- Batas 5 cm dari tembok tetap berlaku. Kalau sebuah kotak gagal dicapai 4 kali, kotak itu dilewati.
- Kotak berarsir (3A–8A, 3B–8B) berada di luar apartemen.
- Konversi kotak ke frame `map` memakai pose start robot dari file world. Kalau posisi start
  robot dipindah di Webots, simpan world-nya. Kalau tidak disimpan, pakai
  `--start-world X Y YAW`.
- Gambar grid bisa dibuat ulang (misalnya setelah world diubah):
  `python3 tools/buat_peta_grid.py --foto`
- Rem driver (`D455_BRAKE_DIST`, 0.40 m) tetap aktif dan hanya menahan gerak **maju**. Akibatnya,
  dari depan robot berhenti sekitar 35 cm dari tembok. Batas 5 cm berlaku untuk samping dan
  belakang, serta saat kamera tidak melihat apa-apa.


### Pilihan (environment variable)

| Variabel | Nilai | Arti |
|---|---|---|
| `D455_PROFILE` | `usb3` (default) | laju datasheet: kamera 30 FPS, gyro 200 Hz* |
| | `usb2` | persis hasil uji USB 2.1: kamera ~11.6 FPS + frame hilang, gyro 21.6 Hz, accel 12.2 Hz |
| `D455_REALISM` | `1` (default) / `0` | noise & artefak kamera asli ON / OFF (FOV & posisi tetap asli) |
| `D455_BRAKE_DIST` | `0.40` | jarak rem anti-tabrak (m) |
| `NO_RVIZ` | `1` | jalankan tanpa RViz2 |
| `WEBOTS_MINIMIZE` | `1` | jendela Webots diperkecil |

Contoh: `D455_PROFILE=usb2 python3 rtabmap_rgbd/launch/run_rtabmap_suite.py`

\* Langkah Webots 16 ms, jadi IMU maksimal 62.5 Hz di simulasi.

### Cek sensor (bandingkan dengan kamera asli)

Saat simulasi berjalan dan **robot diam**, buka terminal lain:

```bash
python3 ~/brone/BRONE_D455_REAL/tools/cek_sensor_d455.py 20
```

Alat ini mengulang pengukuran yang dilakukan pada kamera asli, lalu menampilkan hasil
simulasi dan kamera asli berdampingan.

## 2. Hasil cek sensor (diukur 2026-11-10, profil usb3)

| Besaran | Simulasi | D455 asli |
|---|---|---|
| fx / FOV-H depth | 387.07 / 79.16° | 387.07 / 79.16° |
| fx / FOV-H color | 379.91 / 80.22° | 379.91 / 80.2° |
| Jarak terdekat | 0.293 m | 0.293 m |
| Noise 0.3–1 / 1–2 / 2–3 / 3–4 m | 1.0 / 10.7 / 26.5 / 41.1 mm | 1.0 / 10.8 / 26.4 / 41.0 mm |
| Bias gyro sumbu yaw | −0.00090 rad/s | −0.00089 rad/s |
| Noise gyro yaw | 0.0028 rad/s | 0.0027 rad/s |
| \|g\| accel | 9.739 m/s² | 9.709 m/s² |
| Laju kamera (usb2) | 11.7 Hz | 11.6 Hz |
| Laju gyro (usb2) | 21.4 Hz | 21.6 Hz |

Piksel valid (74 % vs 61 %) dan lebar pita kosong kiri (17 vs 30 px) bergantung pada scene,
karena di uji kamera asli ada kolong meja yang lebih dekat dari 0.3 m.

## 3. Apa saja yang diubah

| Bagian | Simulasi lama | Sekarang | Sumber |
|---|---|---|---|
| FOV depth | 86° | 79.16° (fx 387.07) | uji |
| FOV color | 86° | 80.21° (fx 379.907) | uji |
| Posisi color | sama dengan depth | 59 mm di kanan depth; depth **di-align** ke color | uji (ekstrinsik) |
| Jarak min depth | 0.05–0.15 m | 0.293 m (lebih dekat → 0) | uji |
| Noise depth | nol | interpolasi tabel uji, naik ~Z² | uji |
| Kuantisasi | – | 1 mm (Z16) | datasheet/uji |
| Bayangan stereo | – | dihitung dari baseline 94.97 mm; menghasilkan pita kosong kiri | uji (baseline) |
| Piksel hilang acak | – | 1 % | asumsi |
| IMU orientasi | yaw absolut (InertialUnit) | **tidak ada**; hanya gyro + accel | uji |
| Gyro | sempurna | bias + noise hasil uji | uji |
| Accel | sempurna | noise + \|g\| 0.7 % lebih kecil | uji |
| Yaw odometri | dari InertialUnit | integrasi gyro (ikut drift) | – |
| Rem anti-tabrak | 0.28 m | 0.40 m (kamera buta < 0.293 m) | – |
| Waktu | jam dinding | waktu simulasi (`/clock`, `use_sim_time`) | – |
| EKF | yaw dari IMU | hanya wz dari gyro; bug integrasi ganda diperbaiki | – |
| RTAB-Map | MinDepth 0.15 | `Vis/MinDepth` & `Grid/RangeMin` 0.3 | uji |

Semua angka ada di satu file: `rtabmap_rgbd/src/d455_real_model.py`.

### Topik baru (mengikuti nama di `realsense2_camera`)

- `/camera/color/image_raw` dan `/camera/color/camera_info`
- `/camera/aligned_depth_to_color/image_raw`, yaitu depth yang dipakai VO & RTAB-Map
- `/camera/depth/image_rect_raw` dan `/camera/depth/camera_info`, yaitu depth mentah
- `/imu/data`, berisi gyro + accel dengan `orientation_covariance[0] = -1`
- `/clock`

## 4. Yang belum dimodelkan

- **Distorsi lensa color** dan **titik pusat asli** (cy color 246.6, bukan 239.5): Webots selalu
  merender citra tanpa distorsi dengan pusat di tengah. Citra simulasi setara citra yang sudah di-rectify.
- **Permukaan tanpa tekstur / kaca / hitam**: kamera asli sering kosong di area seperti ini,
  di sini hanya ada dropout acak 1 %.
- **Kamera terbalik**: robot simulasi memasang kamera tegak. Kalau di robot asli dipasang
  terbalik, perlu roll 180° di TF.
- **IMU 200 Hz** dibatasi 62.5 Hz oleh langkah Webots 16 ms.
- Folder `launch/run_slam_simulation.py` (pipeline 2D lama) memakai world baru (FOV baru),
  tetapi **belum** memakai model noise/IMU. Yang disesuaikan penuh hanya suite RTAB-Map.

## 5. Temuan dari simulasi ini

1. **Bug EKF di `BRONE_RESEARCH`**: pesan yang datang terlambat (`/vo/odom`) membuat waktu prediksi
   mundur, sehingga selang waktu yang sama terintegrasi dua kali. Di simulasi lama bug ini tertutupi
   yaw absolut, dan malah "membantu" jarak tempuh. Dengan gyro saja, yaw melenceng sampai 100°+
   dalam 30 detik. Sudah diperbaiki di folder ini (`_predict_to_time`).
2. **Stamp jam dinding** membuat integrasi gyro salah ±10° saat simulasi lebih lambat dari real-time.
   Sekarang semua node memakai waktu simulasi.
3. **Visual Odometry kurang menghitung jarak** dengan noise depth asli. Uji 20 detik auto-explore:

   | Kondisi | VO / jarak asli | Roda / jarak asli |
   |---|---|---|
   | Sim lama | 85 % | 81 % |
   | Sim baru, realisme OFF | 91 % | 81 % |
   | Sim baru, realisme ON (usb3) | **69 %** | 81 % |
   | Sim baru, realisme ON (usb2) | 85 % | 81 % |

   Penyebabnya: VO membuang titik dengan perpindahan > 1.2·dt (≈ 4 cm per frame di 30 FPS). Noise
   depth 2–3 m (≈ 26 mm per frame) membuat batas ini memotong distribusi secara tidak simetris, sehingga
   median tertarik ke nol. Di kamera asli efek ini akan sama. Perbaikan yang disarankan: buang
   outlier di sekitar median, bukan di sekitar nol, dan/atau hanya pakai fitur < 2 m.
4. **Roda omni selip ~19 %** (sudah ada sejak simulasi lama).
5. **Deteksi "roda selip saat menabrak" (stall)** di `layer2_d455_scan.py` memakai ambang 0.4 mm.
   Dengan noise depth asli, ambang ini praktis tidak pernah terpenuhi.
6. **RTAB-Map menolak sebagian loop closure** karena kovariansi odometri (yaw) dari EKF terlalu
   kecil dibanding drift gyro yang sebenarnya (`RGBD/OptimizeMaxError`).

## Cek drift SLAM & loop closure (`tools/cek_drift_slam.py`)

Jalankan di terminal 3 setelah `SELURUH LAYER AKTIF!`, lalu gerakkan robot (teleop/auto-explorer):

```bash
python3 ~/brone/BRONE_D455_REAL/tools/cek_drift_slam.py                 # Ctrl+C untuk selesai
python3 ~/brone/BRONE_D455_REAL/tools/cek_drift_slam.py --duration 600  # stop otomatis (detik sim)
```

Membandingkan pose SLAM (`map->base_link`) dan odometri EKF (`odom->base_link`) dengan
`/ground_truth/pose`, mencatat loop closure/proximity yang diterima dan kandidat yang ditolak,
serta error tiap node graf (`/rtabmap/mapPath`) di akhir. Hasil (`ringkasan.txt`, `sampel.csv`,
`event.csv`, `error_graf.csv`, `grafik_drift.png`) ada di `hasil_cek_drift/<tanggal_jam>/`.
