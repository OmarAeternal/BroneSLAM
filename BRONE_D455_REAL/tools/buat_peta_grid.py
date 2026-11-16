#!/usr/bin/env python3
"""
Membuat gambar grid ruangan (a-h x 1-8) di atas foto atas ruangan Webots.

  python3 tools/buat_peta_grid.py            # pakai foto yang sudah ada di peta_grid/
  python3 tools/buat_peta_grid.py --foto     # ambil ulang foto dari Webots (kalau world diubah)

Hasil (folder peta_grid/):
  foto_ruangan_atas.png : foto atas ruangan dari kamera Webots (1 piksel = 1 cm)
  peta_grid.png         : foto + grid + label kotak + posisi start robot
"""

import math
import os
import shutil
import subprocess
import sys
import tempfile

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(BASE, 'src'))
import room_grid as rg  # noqa: E402

OUT_DIR = os.path.join(BASE, 'peta_grid')
FOTO = os.path.join(OUT_DIR, 'foto_ruangan_atas.png')
HASIL = os.path.join(OUT_DIR, 'peta_grid.png')

# Kamera atas: pusat foto (dunia), skala, ukuran
CX, CY = -6.2, -6.55
M_PER_PX = 0.01
W, H = 1360, 1430
CAM_Z = 200.0


def world_to_px(x, y):
    return W / 2 + (x - CX) / M_PER_PX, H / 2 - (y - CY) / M_PER_PX


def ambil_foto():
    """Salin world ke folder sementara, pasang kamera atas + supervisor, simpan foto."""
    webots = shutil.which('webots')
    if not webots:
        sys.exit('webots tidak ditemukan di PATH')
    tmp = tempfile.mkdtemp(prefix='foto_ruangan_')
    os.makedirs(os.path.join(tmp, 'worlds'))
    os.makedirs(os.path.join(tmp, 'controllers', 'snap'))
    src = open(rg.WORLD_FILE).read().replace('controller "<extern>"', 'controller "<none>"')
    fov = 2 * math.atan((W / 2 * M_PER_PX) / CAM_Z)
    # Kamera Webots melihat +X (atas +Z). Rotasi Rz(90)*Ry(90): lihat ke bawah, atas gambar = +Y dunia
    src += f'''
Robot {{
  translation {CX} {CY} {CAM_Z}
  rotation -0.57735 0.57735 0.57735 2.094395
  children [ Camera {{ name "topcam" fieldOfView {fov:.8f} width {W} height {H} near 150 far 260 }} ]
  name "foto_atas"
  controller "snap"
  supervisor TRUE
}}
'''
    open(os.path.join(tmp, 'worlds', 'snap.wbt'), 'w').write(src)
    out = os.path.join(tmp, 'top.png')
    open(os.path.join(tmp, 'controllers', 'snap', 'snap.py'), 'w').write(f'''
from controller import Supervisor
r = Supervisor(); ts = int(r.getBasicTimeStep())
cam = r.getDevice('topcam'); cam.enable(ts)
for _ in range(20): r.step(ts)
cam.saveImage({out!r}, 100)
r.simulationQuit(0)
''')
    subprocess.run([webots, '--batch', '--mode=fast', '--minimize', '--stdout', '--stderr',
                    os.path.join(tmp, 'worlds', 'snap.wbt')], timeout=180, check=False)
    if not os.path.exists(out):
        sys.exit('Gagal mengambil foto dari Webots')
    os.makedirs(OUT_DIR, exist_ok=True)
    shutil.copy(out, FOTO)
    shutil.rmtree(tmp, ignore_errors=True)
    print('Foto disimpan:', FOTO)


def gambar_grid():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.patches as mp
    import matplotlib.patheffects as pe

    img = plt.imread(FOTO)
    sx, sy, syaw = rg.read_start_pose()
    start_cell = rg.world_to_cell(sx, sy)

    fig = plt.figure(figsize=(W / 100 + 2.2, H / 100 + 2.6), dpi=100)
    ax = fig.add_axes([1.1 / (W / 100 + 2.2), 1.0 / (H / 100 + 2.6),
                       W / 100 / (W / 100 + 2.2), H / 100 / (H / 100 + 2.6)])
    ax.imshow(img, extent=(0, W, H, 0))
    ax.set_xlim(0, W)
    ax.set_ylim(H, 0)
    ax.axis('off')
    halo = [pe.withStroke(linewidth=3, foreground='black')]

    for lab in rg.all_cells():
        x0, x1, y0, y1 = rg.cell_rect_world(lab)
        u0, v0 = world_to_px(x0, y1)
        u1, v1 = world_to_px(x1, y0)
        inside = rg.cell_in_room(lab)
        if not inside:
            ax.add_patch(mp.Rectangle((u0, v0), u1 - u0, v1 - v0, facecolor='black', alpha=0.45,
                                      hatch='//', edgecolor='#888888', linewidth=0))
        if lab == start_cell:
            ax.add_patch(mp.Rectangle((u0, v0), u1 - u0, v1 - v0, facecolor='#ffd400', alpha=0.25))
        ax.text(u0 + 10, v0 + 10, lab.upper(), ha='left', va='top', fontsize=15, fontweight='bold',
                color='white' if inside else '#bbbbbb', path_effects=halo,
                bbox=dict(boxstyle='round,pad=0.2', fc='#1f3b73' if inside else '#444444', ec='none',
                          alpha=0.85))

    # Garis grid
    for i in range(rg.N + 1):
        u, _ = world_to_px(rg.ROOM_X0 + i * rg.CELL_W, 0)
        _, va = world_to_px(0, rg.ROOM_Y1)
        _, vb = world_to_px(0, rg.ROOM_Y0)
        ax.plot([u, u], [va, vb], color='#ff3b30', lw=2.2, alpha=0.9)
        _, v = world_to_px(0, rg.ROOM_Y1 - i * rg.CELL_H)
        ua, _ = world_to_px(rg.ROOM_X0, 0)
        ub, _ = world_to_px(rg.ROOM_X1, 0)
        ax.plot([ua, ub], [v, v], color='#ff3b30', lw=2.2, alpha=0.9)

    # Label sumbu: huruf atas & bawah, angka kiri & kanan
    _, vtop = world_to_px(0, rg.ROOM_Y1)
    _, vbot = world_to_px(0, rg.ROOM_Y0)
    uleft, _ = world_to_px(rg.ROOM_X0, 0)
    uright, _ = world_to_px(rg.ROOM_X1, 0)
    for i, c in enumerate(rg.COLS):
        u, _ = world_to_px(rg.ROOM_X0 + (i + 0.5) * rg.CELL_W, 0)
        for v, va in ((vtop - 12, 'bottom'), (vbot + 12, 'top')):
            ax.text(u, v, c, ha='center', va=va, fontsize=26, fontweight='bold', color='#c00000',
                    clip_on=False)
    for i in range(rg.N):
        _, v = world_to_px(0, rg.ROOM_Y1 - (i + 0.5) * rg.CELL_H)
        for u, ha in ((uleft - 14, 'right'), (uright + 14, 'left')):
            ax.text(u, v, str(i + 1), ha=ha, va='center', fontsize=26, fontweight='bold',
                    color='#c00000', clip_on=False)

    # Robot start + arah hadap
    u, v = world_to_px(sx, sy)
    ax.add_patch(mp.Circle((u, v), 13, fc='#00e05a', ec='black', lw=2, zorder=5))
    du, dv = 55 * math.cos(syaw), -55 * math.sin(syaw)
    ax.annotate('', xy=(u + du, v + dv), xytext=(u, v),
                arrowprops=dict(arrowstyle='-|>', color='#00e05a', lw=3, mutation_scale=25), zorder=6)
    ax.text(u + 20, v - 18, f'START ({start_cell.upper()})', fontsize=14, fontweight='bold',
            color='#00ff66', path_effects=halo, zorder=6)

    # Kompas sumbu dunia Webots
    ox, oy = W - 120, H - 60
    ax.annotate('', xy=(ox + 80, oy), xytext=(ox, oy), arrowprops=dict(arrowstyle='-|>', color='w', lw=2))
    ax.annotate('', xy=(ox, oy - 80), xytext=(ox, oy), arrowprops=dict(arrowstyle='-|>', color='w', lw=2))
    ax.text(ox + 84, oy, '+X', color='w', va='center', fontsize=12, path_effects=halo)
    ax.text(ox, oy - 86, '+Y', color='w', ha='center', fontsize=12, path_effects=halo)

    fig.text(0.5, 1 - 0.25 / (H / 100 + 2.6),
             'BRONE - Grid Ruangan 8x8 (kolom a-h kiri->kanan, baris 1-8 atas->bawah)',
             ha='center', va='top', fontsize=20, fontweight='bold')
    fig.text(0.5, 0.25 / (H / 100 + 2.6),
             f'1 kotak = {rg.CELL_W:.2f} m x {rg.CELL_H:.2f} m   |   arsir = di luar apartemen   |   '
             f'contoh: python3 src/auto_explorer.py --2c   atau   --2c --8h',
             ha='center', va='bottom', fontsize=13)
    os.makedirs(OUT_DIR, exist_ok=True)
    fig.savefig(HASIL, dpi=100)
    print('Peta grid disimpan:', HASIL)


if __name__ == '__main__':
    if '--foto' in sys.argv or not os.path.exists(FOTO):
        ambil_foto()
    gambar_grid()
