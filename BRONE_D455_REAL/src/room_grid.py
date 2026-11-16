#!/usr/bin/env python3
"""
Grid ruangan 8x8 untuk navigasi "pergi ke kotak" (mis. 3e, 8h).

Konvensi (lihat peta_grid/peta_grid.png):
  * Kolom a..h : kiri -> kanan  (X dunia Webots naik)
  * Baris 1..8 : atas -> bawah  (Y dunia Webots turun; baris 1 = sisi Y = 0)
  * Batas grid = garis tengah dinding luar apartemen:
        X dunia -12.40 .. 0.00  (lebar kotak 1.55 m)
        Y dunia -13.12 .. 0.00  (tinggi kotak 1.64 m)

Frame 'map' RTAB-Map = pose robot saat start (x maju, y kiri). Konversi dunia <-> map memakai
pose awal robot yang dibaca dari file world (DEF OMNI_WHEELS Robot).
"""

import math
import os
import re

ROOM_X0, ROOM_X1 = -12.40, 0.00
ROOM_Y0, ROOM_Y1 = -13.12, 0.00
COLS = 'abcdefgh'
N = 8
CELL_W = (ROOM_X1 - ROOM_X0) / N
CELL_H = (ROOM_Y1 - ROOM_Y0) / N

# Denah luar apartemen (bentuk L), dunia Webots
APARTMENT_POLY = [(-12.40, 0.00), (0.00, 0.00), (0.00, -13.12), (-9.90, -13.12),
                  (-9.90, -3.63), (-12.40, -3.63)]

WORLD_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                          'worlds', 'openrmf_warehouse.wbt')

_CELL_RE = re.compile(r'^(?:([1-8])([a-h])|([a-h])([1-8]))$', re.IGNORECASE)


def parse_cell(token: str) -> str:
    """'--2c', '2C', 'c2' -> '2c'. ValueError bila bukan kotak valid."""
    t = token.strip().lstrip('-')
    m = _CELL_RE.match(t)
    if not m:
        raise ValueError(f'kotak tidak valid: {token!r} (contoh: 2c, 8h)')
    row = m.group(1) or m.group(4)
    col = (m.group(2) or m.group(3)).lower()
    return f'{row}{col}'


def is_cell_token(token: str) -> bool:
    try:
        parse_cell(token)
        return True
    except ValueError:
        return False


def cell_rect_world(label: str):
    """-> (x0, x1, y0, y1) dunia Webots."""
    label = parse_cell(label)
    row, col = int(label[0]), COLS.index(label[1])
    x0 = ROOM_X0 + col * CELL_W
    y1 = ROOM_Y1 - (row - 1) * CELL_H
    return x0, x0 + CELL_W, y1 - CELL_H, y1


def cell_center_world(label: str):
    x0, x1, y0, y1 = cell_rect_world(label)
    return 0.5 * (x0 + x1), 0.5 * (y0 + y1)


def world_to_cell(x: float, y: float):
    if not (ROOM_X0 <= x < ROOM_X1 and ROOM_Y0 < y <= ROOM_Y1):
        return None
    col = min(N - 1, int((x - ROOM_X0) / CELL_W))
    row = min(N - 1, int((ROOM_Y1 - y) / CELL_H))
    return f'{row + 1}{COLS[col]}'


def point_in_poly(x, y, poly=APARTMENT_POLY):
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def cell_in_room(label: str) -> bool:
    return point_in_poly(*cell_center_world(label))


def all_cells():
    return [f'{r}{c}' for r in range(1, N + 1) for c in COLS]


def read_start_pose(world_file: str = WORLD_FILE):
    """Pose awal robot (x, y, yaw) dari node 'DEF OMNI_WHEELS Robot' di file world."""
    txt = open(world_file).read()
    i = txt.index('DEF OMNI_WHEELS Robot')
    blk = txt[i:i + 400]
    tr = re.search(r'\n  translation\s+([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)', blk)
    rot = re.search(r'\n  rotation\s+([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)\s+([-\d.eE]+)', blk)
    x, y = float(tr.group(1)), float(tr.group(2))
    yaw = 0.0
    if rot:
        az, ang = float(rot.group(3)), float(rot.group(4))
        yaw = ang * (1.0 if az >= 0 else -1.0)
    return x, y, yaw


class WorldMapTransform:
    """Dunia Webots <-> frame 'map' (asal = pose start robot)."""

    def __init__(self, start=None):
        self.x0, self.y0, self.yaw0 = start if start is not None else read_start_pose()
        self.c, self.s = math.cos(self.yaw0), math.sin(self.yaw0)

    def to_map(self, x, y):
        dx, dy = x - self.x0, y - self.y0
        return self.c * dx + self.s * dy, -self.s * dx + self.c * dy

    def to_world(self, xm, ym):
        return self.x0 + self.c * xm - self.s * ym, self.y0 + self.s * xm + self.c * ym

    def cell_target(self, label: str):
        """Target untuk Explorer: pusat & 4 sudut kotak dalam frame map."""
        label = parse_cell(label)
        x0, x1, y0, y1 = cell_rect_world(label)
        corners = [self.to_map(x, y) for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1))]
        return {'label': label, 'xy': self.to_map(*cell_center_world(label)), 'poly': corners}


if __name__ == '__main__':
    tf = WorldMapTransform()
    print(f'Pose start robot (dunia): x={tf.x0:.2f} y={tf.y0:.2f} yaw={math.degrees(tf.yaw0):.1f} deg '
          f'-> kotak {world_to_cell(tf.x0, tf.y0)}')
    print(f'Kotak {CELL_W:.2f} x {CELL_H:.2f} m')
    for r in range(1, N + 1):
        print(' '.join(f'{r}{c}' if cell_in_room(f'{r}{c}') else '--' for c in COLS))
