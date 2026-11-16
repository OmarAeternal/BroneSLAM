#!/usr/bin/env python3
"""
Inti Auto-Explorer BRONE (tanpa ROS, supaya bisa diuji offline).

Isi:
  GridMap        : OccupancyGrid -> medan jarak ke halangan (distance field)
  GridPlanner    : Dijkstra di grid (sel aman saja) + biaya agar menjauhi tembok
  find_frontiers : batas area diketahui/tidak diketahui (frontier) untuk eksplorasi penuh
  PathFollower   : pengikut jalur (kamera selalu menghadap arah gerak)
  SafetyFilter   : prediksi lintasan 1 s ke depan; perintah ditolak/diperlambat bila
                   jarak tepi robot ke halangan < batas (default 5 cm)
  Explorer       : state machine mode 'frontier' (memetakan seluruh ruangan) dan
                   'random' (jalan acak, tetap memakai peta + lokalisasi)

Semua koordinat di frame 'map' (meter, rad). Jarak "clearance" = jarak TEPI robot
(radius robot_radius) ke permukaan halangan terdekat.
"""

import math
import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


# =============================================================================
# Peta
# =============================================================================
class GridMap:
    OCC_THRESH = 50   # nilai OccupancyGrid >= 50 dianggap halangan

    def __init__(self, data, width, height, res, origin_x, origin_y, pad=4, extra_xy=None):
        g = np.asarray(data, dtype=np.int16).reshape(height, width)
        known = g >= 0
        if known.any():
            # Potong ke kotak area yang sudah diketahui -> komputasi jauh lebih ringan
            ys, xs = np.nonzero(known)
            y0, y1, x0, x1 = ys.min(), ys.max() + 1, xs.min(), xs.max() + 1
        else:
            y0 = y1 = x0 = x1 = 0
        # Perluas sampai titik tambahan (mis. kotak tujuan yang belum terpetakan), boleh di luar peta
        for ex, ey in (extra_xy or []):
            ix = int(math.floor((ex - origin_x) / res))
            iy = int(math.floor((ey - origin_y) / res))
            if y1 == y0:
                y0, y1, x0, x1 = iy, iy + 1, ix, ix + 1
            y0, y1 = min(y0, iy), max(y1, iy + 1)
            x0, x1 = min(x0, ix), max(x1, ix + 1)
        crop = -np.ones((max(y1 - y0, 0), max(x1 - x0, 0)), np.int16)
        sy0, sy1 = max(y0, 0), min(y1, height)
        sx0, sx1 = max(x0, 0), min(x1, width)
        if sy1 > sy0 and sx1 > sx0:
            crop[sy0 - y0:sy1 - y0, sx0 - x0:sx1 - x0] = g[sy0:sy1, sx0:sx1]
        g = np.pad(crop, pad, constant_values=-1)
        self.res = float(res)
        self.ox = origin_x + (x0 - pad) * self.res
        self.oy = origin_y + (y0 - pad) * self.res
        self.h, self.w = g.shape
        self.occ = g >= self.OCC_THRESH
        self.unknown = g < 0
        self.free = ~self.occ & ~self.unknown
        if self.occ.any():
            edt = ndimage.distance_transform_edt(~self.occ) * self.res
        else:
            edt = np.full(g.shape, 99.0)
        # Jarak pusat-sel ke PERMUKAAN sel halangan (konservatif: setengah diagonal sel)
        self.obs_dist = edt - 0.71 * self.res
        self.known_area_m2 = float(self.free.sum()) * self.res ** 2

    def cell_of(self, x, y):
        return int(math.floor((y - self.oy) / self.res)), int(math.floor((x - self.ox) / self.res))

    def inside(self, iy, ix):
        return 0 <= iy < self.h and 0 <= ix < self.w

    def cell_xy(self, iy, ix):
        return self.ox + (ix + 0.5) * self.res, self.oy + (iy + 0.5) * self.res

    def dist_at(self, x, y):
        """Jarak (bilinear) titik (x, y) ke permukaan halangan terdekat. Vektor."""
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        fx = (x - self.ox) / self.res - 0.5
        fy = (y - self.oy) / self.res - 0.5
        cx = np.clip(fx, 0.0, self.w - 1.0)
        cy = np.clip(fy, 0.0, self.h - 1.0)
        # Di luar grid: batas bawah Lipschitz (jarak berkurang paling cepat 1 m/m)
        out = np.hypot(fx - cx, fy - cy) * self.res
        x0 = np.minimum(np.floor(cx).astype(int), self.w - 2) if self.w > 1 else np.zeros_like(cx, int)
        y0 = np.minimum(np.floor(cy).astype(int), self.h - 2) if self.h > 1 else np.zeros_like(cy, int)
        x0 = np.maximum(x0, 0)
        y0 = np.maximum(y0, 0)
        x1 = np.minimum(x0 + 1, self.w - 1)
        y1 = np.minimum(y0 + 1, self.h - 1)
        tx = np.clip(cx - x0, 0.0, 1.0)
        ty = np.clip(cy - y0, 0.0, 1.0)
        d = self.obs_dist
        v = (d[y0, x0] * (1 - tx) * (1 - ty) + d[y0, x1] * tx * (1 - ty)
             + d[y1, x0] * (1 - tx) * ty + d[y1, x1] * tx * ty)
        return v - out


# =============================================================================
# Perencana jalur
# =============================================================================
class GridPlanner:
    def __init__(self, robot_radius, hard_clear, pref_clear, clear_weight=4.0):
        self.R = robot_radius
        self.hard = hard_clear
        self.pref = max(pref_clear, hard_clear + 0.01)
        self.w = clear_weight
        self.gm = None
        self.dist2d = None

    def plan_from(self, gm: GridMap, x, y, allow_unknown=False, unknown_cost=0.6):
        """Dijkstra dari posisi robot ke SEMUA sel aman. Hasil: self.dist2d (m, inf = tak terjangkau).
        allow_unknown: sel belum terpetakan boleh dilewati (optimistis, biaya lebih mahal); dipakai
        mode goto. Aman karena robot selalu menghadap arah gerak & jalur direncanakan ulang tiap 2 s."""
        self.gm = gm
        clearance = gm.obs_dist - self.R
        passable = (gm.free | gm.unknown) if allow_unknown else gm.free
        trav = passable & (clearance >= self.hard)

        sy, sx = gm.cell_of(x, y)
        sy = min(max(sy, 0), gm.h - 1)
        sx = min(max(sx, 0), gm.w - 1)
        # Sel di sekitar/bawah robot boleh 'unknown' (kamera buta < 0.29 m)
        r_cells = int(math.ceil((self.R + 0.10) / gm.res))
        yy, xx = np.ogrid[:gm.h, :gm.w]
        near = (yy - sy) ** 2 + (xx - sx) ** 2 <= r_cells ** 2
        start_clear = clearance[sy, sx]
        trav |= near & ~gm.occ & (clearance >= min(self.hard, start_clear))
        trav[sy, sx] = True

        # Biaya: makin dekat tembok (di bawah pref_clear) makin mahal -> jalur lewat tengah
        pen = np.clip((self.pref - clearance) / (self.pref - self.hard), 0.0, 1.0) ** 2
        cost = 1.0 + self.w * pen
        if allow_unknown:
            cost = cost + unknown_cost * gm.unknown

        idx = -np.ones(trav.shape, dtype=np.int64)
        n = int(trav.sum())
        idx[trav] = np.arange(n)
        rows, cols, wts = [], [], []
        h, w = trav.shape
        for dy, dx in ((0, 1), (1, 0), (1, 1), (1, -1)):
            ya0, ya1 = max(0, -dy), h - max(0, dy)
            xa0, xa1 = max(0, -dx), w - max(0, dx)
            tA = trav[ya0:ya1, xa0:xa1]
            tB = trav[ya0 + dy:ya1 + dy, xa0 + dx:xa1 + dx]
            m = tA & tB
            if not m.any():
                continue
            cA = cost[ya0:ya1, xa0:xa1][m]
            cB = cost[ya0 + dy:ya1 + dy, xa0 + dx:xa1 + dx][m]
            rows.append(idx[ya0:ya1, xa0:xa1][m])
            cols.append(idx[ya0 + dy:ya1 + dy, xa0 + dx:xa1 + dx][m])
            wts.append(0.5 * (cA + cB) * math.hypot(dx, dy) * gm.res)
        self.idx = idx
        self.dist2d = np.full(trav.shape, np.inf)
        if n == 0 or not rows:
            self.pred = None
            self.dist2d[sy, sx] = 0.0
            return
        G = coo_matrix((np.concatenate(wts), (np.concatenate(rows), np.concatenate(cols))),
                       shape=(n, n)).tocsr()
        dist, pred = dijkstra(G, directed=False, indices=int(idx[sy, sx]), return_predecessors=True)
        self.pred = pred
        self.dist2d[trav] = dist
        self.trav = trav
        self.start = (sy, sx)

    def path_to(self, iy, ix):
        """Daftar titik (N,2) dari robot ke sel (iy, ix); None bila tak terjangkau."""
        gm = self.gm
        if gm is None or not gm.inside(iy, ix) or not np.isfinite(self.dist2d[iy, ix]):
            return None
        node = int(self.idx[iy, ix])
        if self.pred is None:
            return np.array([gm.cell_xy(iy, ix)])
        flat = np.flatnonzero(self.idx.ravel() >= 0)  # sel ke-n -> indeks flat
        order = []
        while node >= 0:
            order.append(node)
            node = int(self.pred[node])
        cells = flat[np.array(order[::-1])]
        py, px = np.divmod(cells, gm.w)
        return np.column_stack((gm.ox + (px + 0.5) * gm.res, gm.oy + (py + 0.5) * gm.res))

    def reachable_cell_near(self, x, y, radius=0.25):
        """Sel terjangkau terdekat dari titik (x, y) dalam radius; None bila tidak ada."""
        gm = self.gm
        iy, ix = gm.cell_of(x, y)
        r = int(math.ceil(radius / gm.res))
        y0, y1 = max(0, iy - r), min(gm.h, iy + r + 1)
        x0, x1 = max(0, ix - r), min(gm.w, ix + r + 1)
        if y0 >= y1 or x0 >= x1:
            return None
        sub = self.dist2d[y0:y1, x0:x1]
        yy, xx = np.mgrid[y0:y1, x0:x1]
        dd = np.hypot(yy - iy, xx - ix) * gm.res
        ok = np.isfinite(sub) & (dd <= radius)
        if not ok.any():
            return None
        k = np.argmin(np.where(ok, dd, np.inf))
        return int(yy.ravel()[k]), int(xx.ravel()[k])


# =============================================================================
# Frontier
# =============================================================================
def _cell_keys(gm: GridMap, ys, xs):
    gx = np.round((gm.ox + (xs + 0.5) * gm.res) / gm.res).astype(np.int64)
    gy = np.round((gm.oy + (ys + 0.5) * gm.res) / gm.res).astype(np.int64)
    return gx * 1_000_003 + gy


def frontier_mask(gm: GridMap, ignored_keys=None, min_size_cells=4):
    unk_nb = ndimage.binary_dilation(gm.unknown, structure=ndimage.generate_binary_structure(2, 1))
    frontier = gm.free & unk_nb
    if ignored_keys is not None and len(ignored_keys) and frontier.any():
        ys, xs = np.nonzero(frontier)
        drop = np.isin(_cell_keys(gm, ys, xs), ignored_keys)
        frontier[ys[drop], xs[drop]] = False
    lab, n = ndimage.label(frontier, structure=np.ones((3, 3)))
    if n == 0:
        return lab, np.zeros(0)
    sizes = np.asarray(ndimage.sum(frontier, lab, index=np.arange(1, n + 1)))
    small = np.concatenate(([True], sizes < min_size_cells))
    lab[small[lab]] = 0
    return lab, sizes


def find_frontier_goals(gm, planner, ignored_keys, blacklist, goal_radius=0.6,
                        min_size_cells=4, size_bonus=0.6, blacklist_r=0.35):
    """Satu tujuan (sel terjangkau termurah dekat frontier) per cluster frontier, urut skor."""
    lab, sizes = frontier_mask(gm, ignored_keys, min_size_cells)
    big = lab > 0
    if not big.any():
        return []
    dist_f, (ny, nx) = ndimage.distance_transform_edt(~big, return_indices=True)
    cand = np.isfinite(planner.dist2d) & (dist_f * gm.res <= goal_radius)
    cy, cx = np.nonzero(cand)
    if cy.size == 0:
        return []
    if blacklist:
        wx = gm.ox + (cx + 0.5) * gm.res
        wy = gm.oy + (cy + 0.5) * gm.res
        keep = np.ones(cy.size, bool)
        for bx, by in blacklist:
            keep &= np.hypot(wx - bx, wy - by) > blacklist_r
        cy, cx = cy[keep], cx[keep]
        if cy.size == 0:
            return []
    clab = lab[ny[cy, cx], nx[cy, cx]]
    cost = planner.dist2d[cy, cx]
    order = np.lexsort((cost, clab))
    _, first = np.unique(clab[order], return_index=True)
    goals = []
    for k in order[first]:
        L = int(clab[k])
        size_m = float(sizes[L - 1]) * gm.res
        score = float(cost[k]) - size_bonus * min(size_m, 1.5)
        gx, gy = gm.cell_xy(int(cy[k]), int(cx[k]))
        goals.append({'cell': (int(cy[k]), int(cx[k])), 'xy': (gx, gy), 'cost': float(cost[k]),
                      'size_m': size_m, 'score': score, 'label': L})
    goals.sort(key=lambda g: g['score'])
    return goals


def frontier_near(gm, ignored_keys, x, y, radius, min_size_cells=4):
    lab, _ = frontier_mask(gm, ignored_keys, min_size_cells)
    ys, xs = np.nonzero(lab > 0)
    if ys.size == 0:
        return False
    wx = gm.ox + (xs + 0.5) * gm.res
    wy = gm.oy + (ys + 0.5) * gm.res
    return bool(np.any(np.hypot(wx - x, wy - y) <= radius))


def frontier_keys_near(gm, x, y, radius):
    lab, _ = frontier_mask(gm, None, 1)
    ys, xs = np.nonzero(lab > 0)
    if ys.size == 0:
        return np.zeros(0, np.int64)
    wx = gm.ox + (xs + 0.5) * gm.res
    wy = gm.oy + (ys + 0.5) * gm.res
    m = np.hypot(wx - x, wy - y) <= radius
    return _cell_keys(gm, ys[m], xs[m])


def points_in_poly(px, py, poly):
    px = np.asarray(px, float)
    py = np.asarray(py, float)
    inside = np.zeros(px.shape, bool)
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if y1 == y2:
            continue
        cross = ((y1 > py) != (y2 > py)) & (px < (x2 - x1) * (py - y1) / (y2 - y1) + x1)
        inside ^= cross
    return inside


# =============================================================================
# Pengikut jalur & filter keselamatan
# =============================================================================
class PathFollower:
    def __init__(self, max_v=0.22, max_w=0.6, lookahead=0.35, turn_in_place=0.7, k_w=1.6):
        self.max_v, self.max_w = max_v, max_w
        self.lookahead, self.turn_in_place, self.k_w = lookahead, turn_in_place, k_w
        self.path = None
        self.i = 0

    def set_path(self, path):
        self.path = path
        self.i = 0

    def command(self, x, y, th):
        """-> (vx, wz, jarak_ke_ujung_jalur). Robot holonomik tapi tetap 'menghadap' arah gerak,
        karena D455 hanya melihat ke depan (FOV 79 deg)."""
        p = self.path
        if p is None or len(p) == 0:
            return 0.0, 0.0, 0.0
        win = p[self.i:self.i + 40]
        self.i += int(np.argmin(np.hypot(win[:, 0] - x, win[:, 1] - y)))
        rest = p[self.i:]
        d = np.hypot(rest[:, 0] - x, rest[:, 1] - y)
        far = np.nonzero(d >= self.lookahead)[0]
        tx, ty = rest[far[0]] if far.size else rest[-1]
        dist_goal = float(math.hypot(p[-1, 0] - x, p[-1, 1] - y))
        ang = wrap(math.atan2(ty - y, tx - x) - th)
        w = max(-self.max_w, min(self.max_w, self.k_w * ang))
        if abs(ang) > self.turn_in_place:
            return 0.0, w, dist_goal
        v = self.max_v * max(0.0, math.cos(ang)) ** 2
        v = min(v, 0.8 * dist_goal + 0.05)
        return v, w, dist_goal


class SafetyFilter:
    """Menjamin jarak tepi robot ke halangan >= hard_clear sepanjang prediksi lintasan.

    Sumber halangan:
      1. peta RTAB-Map (distance field, frame map, pakai pose lokalisasi)
      2. memori titik /scan beberapa detik terakhir (frame base_link) untuk halangan yang
         sudah masuk zona buta kamera (< 0.293 m) atau belum masuk peta.
    Robot berbentuk lingkaran -> putar di tempat tidak pernah mengubah jarak.
    """

    def __init__(self, robot_radius, hard_clear, horizon=1.0, dt=0.1, slow_clear=0.35):
        self.R, self.hard = robot_radius, hard_clear
        self.horizon, self.dt = horizon, dt
        self.slow_clear = slow_clear

    def _rollout(self, vx, vy, wz):
        n = max(1, int(round(self.horizon / self.dt)))
        k = np.arange(1, n + 1) * self.dt
        th = wz * k
        if abs(wz) < 1e-6:
            rx, ry = vx * k, vy * k
        else:
            rx = (vx * np.sin(th) + vy * (np.cos(th) - 1.0)) / wz
            ry = (vx * (1.0 - np.cos(th)) + vy * np.sin(th)) / wz
        return np.concatenate(([0.0], rx)), np.concatenate(([0.0], ry))

    def clearance_rel(self, gm, pose, pts_base, rx, ry):
        x, y, th = pose
        c, s = math.cos(th), math.sin(th)
        mx = x + c * rx - s * ry
        my = y + s * rx + c * ry
        clr = (gm.dist_at(mx, my) if gm is not None else np.full(np.shape(rx), 99.0)) - self.R
        if pts_base is not None and len(pts_base):
            dp = np.hypot(pts_base[None, :, 0] - np.atleast_1d(rx)[:, None],
                          pts_base[None, :, 1] - np.atleast_1d(ry)[:, None]).min(axis=1) - self.R
            clr = np.minimum(clr, dp)
        return clr

    def clearance_now(self, gm, pose, pts_base):
        return float(self.clearance_rel(gm, pose, pts_base, np.array([0.0]), np.array([0.0]))[0])

    def filter(self, gm, pose, pts_base, vx, vy, wz):
        """-> (vx, vy, wz, status). status: 'ok' | 'slow' | 'blocked' | 'escape'."""
        now = self.clearance_now(gm, pose, pts_base)
        # Batas kecepatan sebanding jarak (makin dekat tembok makin pelan)
        lim = float(np.interp(now, [self.hard, self.slow_clear], [0.05, 1.0]))
        sp = math.hypot(vx, vy)
        if sp > 1e-9:
            cap = max(0.04, lim * 0.25)
            if sp > cap and lim < 1.0:
                vx, vy = vx * cap / sp, vy * cap / sp
        for scale in (1.0, 0.6, 0.35, 0.15):
            rx, ry = self._rollout(vx * scale, vy * scale, wz)
            clr = self.clearance_rel(gm, pose, pts_base, rx, ry)
            if clr.min() >= self.hard:
                return vx * scale, vy * scale, wz, ('ok' if scale == 1.0 else 'slow')
            # Sudah terlanjur di bawah batas (peta bergeser): hanya boleh gerak yang menjauh
            if now < self.hard and clr.min() >= now - 0.003 and clr[-1] > now + 0.01:
                return vx * scale, vy * scale, wz, 'escape'
        return 0.0, 0.0, wz, 'blocked'

    def escape_dir(self, gm, pose, pts_base, eps=0.03):
        """Arah (frame base) menjauhi halangan terdekat (gradien jarak)."""
        rx = np.array([eps, -eps, 0.0, 0.0])
        ry = np.array([0.0, 0.0, eps, -eps])
        c = self.clearance_rel(gm, pose, pts_base, rx, ry)
        g = np.array([c[0] - c[1], c[2] - c[3]])
        n = np.linalg.norm(g)
        return (g / n) if n > 1e-6 else np.array([-1.0, 0.0])


# =============================================================================
# State machine
# =============================================================================
class ExplorerConfig:
    def __init__(self, **kw):
        self.mode = 'frontier'          # 'frontier' | 'random' | 'goto'
        self.targets = []               # mode goto: [{'label','xy','poly'}] di frame map
        self.robot_radius = 0.13        # badan r=0.10 + roda omni (m)
        self.min_clearance = 0.05       # batas jarak TEPI robot ke tembok (m)
        self.pose_margin = 0.03         # cadangan error lokalisasi/peta (m)
        self.pref_clearance = 0.30      # jarak yang disukai perencana (lewat tengah lorong)
        self.max_v = 0.22
        self.max_w = 0.6
        self.spin_w = 0.45
        self.spin_at_goal = True
        self.return_home = True
        self.goal_tol = 0.15
        self.goal_radius = 0.6          # tujuan frontier: sel aman dalam radius ini dari frontier
        self.replan_s = 2.0
        self.stuck_s = 6.0
        self.random_min_d = 0.8
        self.random_max_d = 4.0
        self.horizon = 1.0
        self.goto_attempts = 4          # goto: percobaan ulang sebelum kotak dilewati
        for k, v in kw.items():
            if not hasattr(self, k):
                raise ValueError(f'parameter tidak dikenal: {k}')
            setattr(self, k, v)

    @property
    def plan_clear(self):
        return self.min_clearance + self.pose_margin


class Explorer:
    def __init__(self, cfg: ExplorerConfig, rng=None, log=print):
        self.cfg = cfg
        self.rng = rng if rng is not None else np.random.default_rng()
        self.log = log
        self.planner = GridPlanner(cfg.robot_radius, cfg.plan_clear, cfg.pref_clearance)
        self.follower = PathFollower(cfg.max_v, cfg.max_w)
        # Filter memakai min_clearance + setengah margin: perencana sudah menjaga margin penuh
        self.safety = SafetyFilter(cfg.robot_radius, cfg.min_clearance + 0.5 * cfg.pose_margin,
                                   horizon=cfg.horizon)
        self.gm = None
        self.state = 'INIT_SPIN'
        self.home = None
        self.goal = None
        self.path = None
        self.blacklist = []
        self.ignored = np.zeros(0, np.int64)
        self.goals_done = 0
        self.min_clear_seen = float('inf')
        self.last_status = ''
        self._spin_acc = 0.0
        self._spin_last = None
        self._t_plan = -1e9
        self._t_goal = 0.0
        self._prog_t = 0.0
        self._prog_pose = None
        self._block_since = None
        self._goto_tries = 0

    # ------------------------------------------------------------------ utils
    def update_map(self, gm: GridMap):
        self.gm = gm

    def _start_spin(self, pose, state='SPIN'):
        self.state = state
        self._spin_acc = 0.0
        self._spin_last = pose[2]

    def _spin(self, pose):
        self._spin_acc += abs(wrap(pose[2] - self._spin_last))
        self._spin_last = pose[2]
        return self._spin_acc >= 2 * math.pi * 0.97

    def _set_goal(self, t, pose, goal_xy, path):
        self.goal = goal_xy
        self.path = path
        self.follower.set_path(path)
        self.state = 'FOLLOW'
        self._t_plan = t
        self._t_goal = t
        self._prog_t, self._prog_pose = t, pose
        self._block_since = None

    def _select(self, t, pose):
        gm, cfg = self.gm, self.cfg
        if gm is None:
            return False
        if cfg.mode == 'goto':
            if not cfg.targets:
                return None
            self.planner.plan_from(gm, pose[0], pose[1], allow_unknown=True)
            tgt = cfg.targets[0]
            cell = self._goto_goal_cell()
            path = self.planner.path_to(*cell) if cell is not None else None
            if path is None:
                self._goto_fail('tidak ada jalur aman ke kotak ini (untuk sekarang)')
                return False
            gxy = gm.cell_xy(*cell)
            self.log(f"[goto] menuju kotak {tgt['label'].upper()} -> titik ({gxy[0]:.2f}, {gxy[1]:.2f}) "
                     f"frame map, jarak jalur {self.planner.dist2d[cell]:.2f} m")
            self._set_goal(t, pose, gxy, path)
            return True
        self.planner.plan_from(gm, pose[0], pose[1])
        if cfg.mode == 'frontier':
            goals = find_frontier_goals(gm, self.planner, self.ignored, self.blacklist,
                                        goal_radius=cfg.goal_radius)
            if not goals:
                return None   # eksplorasi selesai
            g = goals[0]
            path = self.planner.path_to(*g['cell'])
            if path is None:
                self.blacklist.append(g['xy'])
                return False
            self.log(f"[frontier] tujuan ({g['xy'][0]:.2f}, {g['xy'][1]:.2f}) jarak {g['cost']:.2f} m, "
                     f"frontier {g['size_m']:.2f} m, sisa cluster {len(goals)}")
            self._set_goal(t, pose, g['xy'], path)
            return True
        # ---- mode random: titik acak di area aman yang sudah dipetakan
        d = self.planner.dist2d
        clr = gm.obs_dist - cfg.robot_radius
        ok = np.isfinite(d) & (d >= cfg.random_min_d) & (d <= cfg.random_max_d) & gm.free
        open_ok = ok & (clr >= cfg.pref_clearance)
        pick = open_ok if open_ok.any() else ok
        ys, xs = np.nonzero(pick)
        if ys.size == 0:
            return False
        k = int(self.rng.integers(ys.size))
        path = self.planner.path_to(int(ys[k]), int(xs[k]))
        if path is None:
            return False
        gxy = gm.cell_xy(int(ys[k]), int(xs[k]))
        self.log(f'[random] tujuan acak ({gxy[0]:.2f}, {gxy[1]:.2f}), jarak jalur {d[ys[k], xs[k]]:.2f} m')
        self._set_goal(t, pose, gxy, path)
        return True

    def _replan(self, t, pose):
        self._t_plan = t
        if self.cfg.mode == 'goto' and self.state == 'FOLLOW':
            self.planner.plan_from(self.gm, pose[0], pose[1], allow_unknown=True)
            cell = self._goto_goal_cell()
            if cell is not None:
                self.goal = self.gm.cell_xy(*cell)
        else:
            self.planner.plan_from(self.gm, pose[0], pose[1])
            cell = self.planner.reachable_cell_near(self.goal[0], self.goal[1], 0.25)
        path = self.planner.path_to(*cell) if cell is not None else None
        if path is None:
            return False
        self.path = path
        self.follower.set_path(path)
        return True

    def _goto_goal_cell(self):
        """Titik tujuan di DALAM kotak target: terjangkau & aman, sedekat mungkin ke pusat kotak,
        lebih suka area yang sudah terpetakan dan lega (mis. pusat kotak = kasur -> pinggir kasur)."""
        gm, cfg = self.gm, self.cfg
        tgt = cfg.targets[0]
        ys, xs = np.nonzero(np.isfinite(self.planner.dist2d))
        if ys.size == 0:
            return None
        wx = gm.ox + (xs + 0.5) * gm.res
        wy = gm.oy + (ys + 0.5) * gm.res
        ok = points_in_poly(wx, wy, tgt['poly'])
        for bx, by in self.blacklist:
            ok &= np.hypot(wx - bx, wy - by) > 0.35
        if not ok.any():
            return None
        ys, xs, wx, wy = ys[ok], xs[ok], wx[ok], wy[ok]
        clr = gm.obs_dist[ys, xs] - cfg.robot_radius
        score = (np.hypot(wx - tgt['xy'][0], wy - tgt['xy'][1])
                 + 0.6 * gm.unknown[ys, xs]
                 + 0.8 * np.clip(cfg.pref_clearance - clr, 0.0, None) / cfg.pref_clearance)
        k = int(np.argmin(score))
        return int(ys[k]), int(xs[k])

    def _goto_fail(self, why):
        self._goto_tries += 1
        tgt = self.cfg.targets[0]
        if self._goto_tries >= self.cfg.goto_attempts:
            self.log(f"[goto] kotak {tgt['label'].upper()} DILEWATI setelah {self._goto_tries} percobaan: {why}")
            self.cfg.targets.pop(0)
            self._goto_tries = 0
            self.blacklist = []
        else:
            self.log(f"[goto] kotak {tgt['label'].upper()}: {why} (percobaan {self._goto_tries}/"
                     f"{self.cfg.goto_attempts})")

    def _abandon(self, why):
        self.log(f'[{self.cfg.mode}] tujuan dibatalkan: {why}')
        if self.goal is not None:
            self.blacklist.append(self.goal)
        if self.cfg.mode == 'goto' and self.cfg.targets:
            self._goto_fail(why)
        self.goal = None
        self.state = 'SELECT'

    def _goal_finished(self, t, pose):
        self.goals_done += 1
        if self.cfg.mode == 'frontier':
            # Frontier yang masih tersisa di dekat tujuan setelah robot melihat sekeliling
            # dianggap tidak bisa dilihat (bayangan/terlalu dekat) -> diabaikan
            keys = frontier_keys_near(self.gm, self.goal[0], self.goal[1], self.cfg.goal_radius + 0.4)
            self.ignored = np.union1d(self.ignored, keys)
        self.goal = None
        self.state = 'SELECT'

    # ------------------------------------------------------------------ main
    def step(self, t, pose, pts_base=None):
        """pose = (x, y, yaw) di frame map. pts_base = titik halangan (N,2) di frame base_link.
        -> (vx, vy, wz)"""
        cfg = self.cfg
        if self.home is None:
            self.home = (pose[0], pose[1])
            self._start_spin(pose, 'INIT_SPIN')

        now_clear = self.safety.clearance_now(self.gm, pose, pts_base)
        self.min_clear_seen = min(self.min_clear_seen, now_clear)

        vx = vy = wz = 0.0
        st = self.state
        if st in ('INIT_SPIN', 'SPIN'):
            if self._spin(pose):
                if st == 'SPIN':
                    self._goal_finished(t, pose)
                else:
                    self.state = 'SELECT'
            else:
                wz = cfg.spin_w
        elif st == 'SELECT':
            r = self._select(t, pose)
            if r is None and cfg.mode == 'goto':
                self.log('[goto] SEMUA KOTAK TUJUAN SELESAI.')
                self.state = 'DONE'
            elif r is None:
                if cfg.return_home and self.home is not None and \
                        math.hypot(pose[0] - self.home[0], pose[1] - self.home[1]) > cfg.goal_tol:
                    self.log('[frontier] Tidak ada frontier lagi -> PETA SELESAI. Kembali ke titik awal.')
                    self.planner.plan_from(self.gm, pose[0], pose[1])
                    cell = self.planner.reachable_cell_near(self.home[0], self.home[1], 0.3)
                    path = self.planner.path_to(*cell) if cell else None
                    if path is not None:
                        self._set_goal(t, pose, self.home, path)
                        self.state = 'HOME'
                        return 0.0, 0.0, 0.0
                self.log('[frontier] EKSPLORASI SELESAI.')
                self.state = 'DONE'
            elif r is False:
                # Belum ada tujuan (peta masih kosong / semua diblacklist sementara): lihat sekeliling
                wz = cfg.spin_w * 0.7
                if len(self.blacklist) > 30:
                    self.blacklist = self.blacklist[-15:]
        elif st in ('FOLLOW', 'HOME'):
            x, y, th = pose
            if t - self._t_plan > cfg.replan_s:
                if not self._replan(t, pose):
                    if st == 'HOME':
                        self.state = 'DONE'
                        self.log('[frontier] Titik awal tidak terjangkau. Berhenti.')
                    else:
                        self._abandon('jalur hilang setelah peta diperbarui')
                    return 0.0, 0.0, 0.0
                if st == 'FOLLOW' and cfg.mode == 'frontier' and not frontier_near(
                        self.gm, self.ignored, self.goal[0], self.goal[1], cfg.goal_radius + 0.25):
                    self.log('[frontier] Area tujuan sudah terpetakan dalam perjalanan -> pilih frontier baru')
                    self._goal_finished(t, pose)
                    return 0.0, 0.0, 0.0
            v, w, dgoal = self.follower.command(x, y, th)
            if dgoal < cfg.goal_tol:
                if st == 'HOME':
                    self.log('[frontier] Sampai di titik awal. EKSPLORASI SELESAI.')
                    self.state = 'DONE'
                elif cfg.mode == 'goto':
                    tgt = cfg.targets.pop(0)
                    self.goals_done += 1
                    self.log(f"[goto] SAMPAI di kotak {tgt['label'].upper()} "
                             f"(pose map {x:.2f}, {y:.2f}). Sisa tujuan: "
                             f"{', '.join(g['label'].upper() for g in cfg.targets) or '-'}")
                    self._goto_tries = 0
                    self.blacklist = []
                    self.goal = None
                    self.state = 'SELECT'
                elif cfg.mode == 'frontier' and cfg.spin_at_goal:
                    self._start_spin(pose, 'SPIN')
                else:
                    self._goal_finished(t, pose)
                return 0.0, 0.0, 0.0
            vx, wz = v, w
            # Deteksi macet (mis. rem anti-tabrak driver / halangan baru)
            if self._prog_pose is not None:
                moved = math.hypot(x - self._prog_pose[0], y - self._prog_pose[1])
                turned = abs(wrap(th - self._prog_pose[2]))
                if moved > 0.08 or turned > 0.5:
                    self._prog_t, self._prog_pose = t, pose
                elif t - self._prog_t > cfg.stuck_s:
                    if st == 'HOME':
                        self.state = 'DONE'
                        self.log('[frontier] Macet saat pulang. Berhenti.')
                    else:
                        self._abandon('robot tidak bergerak (macet)')
                    return 0.0, 0.0, 0.0
        # DONE -> diam

        # ---------------- filter keselamatan (berlaku di semua state)
        if now_clear < self.safety.hard and st != 'DONE':
            e = self.safety.escape_dir(self.gm, pose, pts_base) * 0.06
            vx, vy, wz = float(e[0]), float(e[1]), 0.0
        vx, vy, wz, status = self.safety.filter(self.gm, pose, pts_base, vx, vy, wz)
        self.last_status = status
        return vx, vy, wz
