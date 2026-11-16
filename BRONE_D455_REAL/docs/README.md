# Dokumentasi BRONE_D455_REAL

## Diagram arsitektur pipeline
| File | Isi |
|---|---|
| `arsitektur_pipeline.html` | Halaman diagram lengkap, tema terang/gelap mengikuti sistem |
| `arsitektur_pipeline_light.html` | Versi terang bergaya laporan (Gambar 1–5, tabel parameter) |
| `gambar/01_gambaran_umum.{svg,png}` | Gambar 1: aliran data antar-node |
| `gambar/02_slam.{svg,png}` | Gambar 2: alur SLAM di RTAB-Map (node, loop closure, optimasi) |
| `gambar/03_mapping.{svg,png}` | Gambar 3: alur pembuatan peta (grid lokal → /map) |
| `gambar/04_tembok_dobel.{svg,png}` | Gambar 4: asal tembok dobel pada peta |
| `gambar/05_lokalisasi.{svg,png}` | Gambar 5: sumber pose robot dan pohon TF |

PNG dirender 2× (resolusi tinggi) untuk dimasukkan ke laporan; SVG bisa diedit di Inkscape.
Kedua halaman HTML memuat font dari Google Fonts (perlu internet); tanpa internet tetap tampil dengan font cadangan.

## Hasil cek drift SLAM (`../hasil_cek_drift/`)
Dibuat oleh `tools/cek_drift_slam.py` (auto-explorer mode random, seed 1, 600 s waktu sim).

| Folder | Kondisi | Catatan |
|---|---|---|
| `20261114_210833` | sebelum perbaikan VO | angka loop closure & error node **tidak valid** (bug skrip, sudah diperbaiki) |
| `20261114_212148` | sebelum perbaikan VO | error SLAM rata-rata 129 cm, robot macet di t≈233 s |
| `20261114_215241` | sesudah perbaikan VO | error SLAM rata-rata 18 cm, error node peta median 7 cm |

## Perbaikan VO (14 November 2026)
`rtabmap_rgbd/src/rgbd_visual_odometry.py`: gerak diestimasi sebagai transformasi rigid 2D
(rotasi + translasi, RANSAC + Kabsch) di frame `base_link`, bukan lagi selisih titik fitur.
Metode lama membaca putaran di tempat sebagai geser samping ±1,25 m/s.
Uji sintetis: `python3 tools/uji_vo_rigid.py`.
