#!/usr/bin/env python3
"""
Cek drift SLAM dan loop closure RTAB-Map terhadap ground truth Webots.

Yang diukur selama robot berjalan:
  - Error pose SLAM  (TF map->base_link)  vs /ground_truth/pose
  - Error pose ODOM  (TF odom->base_link, EKF) vs /ground_truth/pose
    -> kalau error SLAM ~ error ODOM, berarti SLAM tidak mengoreksi apa-apa.
  - Loop closure (loop_closure_id) dan proximity (proximity_detection_id) yang DITERIMA
  - Kandidat loop closure yang DITOLAK (Loop/Highest_hypothesis_id > 0 tapi tidak diterima,
    atau Loop/RejectedHypothesis = 1)
  - Lompatan koreksi map->odom (tanda peta "dibetulkan" oleh loop closure)
  - Di akhir: error SELURUH node graf teroptimasi (/mapGraph) vs ground truth.
    Ini yang paling menentukan kualitas peta: kalau node lama posisinya salah,
    tembok dari node itu tergambar di tempat yang salah (tembok dobel/hantu).

Pakai (jalankan setelah run_rtabmap_suite.py aktif, lalu gerakkan robot):
  python3 tools/cek_drift_slam.py                  # berhenti dengan Ctrl+C
  python3 tools/cek_drift_slam.py --duration 600   # berhenti otomatis setelah 600 s (waktu sim)
Hasil: CSV + grafik PNG + ringkasan di folder hasil_cek_drift/<tanggal_jam>/
"""

import argparse
import bisect
import csv
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from rclpy.time import Time
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rtabmap_msgs.msg import Info, MapGraph

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def ang_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def stat(info, key, default=None):
    for k, v in zip(info.stats_keys, info.stats_values):
        if k == key:
            return v
    return default


class CekDrift(Node):
    def __init__(self, args):
        super().__init__('cek_drift_slam',
                         parameter_overrides=[Parameter('use_sim_time', value=not args.wall_time)])
        self.args = args
        self.tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        self.gt_hist = []       # (t, x, y, yaw)
        self.gt_t = []          # hanya t, untuk bisect
        self.rows = []          # sampel periodik
        self.events = []        # (t, jenis, keterangan)
        self.dist = 0.0
        self.last_gt_xy = None
        self.n_lc = 0
        self.n_prox = 0
        self.n_rej = 0
        self.n_info = 0
        self.last_m2o = None
        self.max_jump = 0.0
        self.map_graph = None
        self.node_stamp = {}    # id node -> stamp (dari /info)
        self.t0 = None
        self.last_print = 0.0
        self.stop = False

        self.create_subscription(PoseStamped, '/ground_truth/pose', self.gt_cb, 50)
        self.create_subscription(Info, args.info_topic, self.info_cb, 20)
        self.create_subscription(MapGraph, args.graph_topic, self.graph_cb, 5)
        self.create_timer(args.period, self.sample)

    # ------------------------------------------------------------------ util
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def rel_t(self, t):
        return t - self.t0 if self.t0 is not None else 0.0

    def gt_at(self, t, max_dt=0.25):
        """Ground truth terdekat ke waktu t (interpolasi linier)."""
        if not self.gt_t:
            return None
        i = bisect.bisect_left(self.gt_t, t)
        if i <= 0:
            g = self.gt_hist[0]
            return g if abs(g[0] - t) <= max_dt else None
        if i >= len(self.gt_t):
            g = self.gt_hist[-1]
            return g if abs(g[0] - t) <= max_dt else None
        a, b = self.gt_hist[i - 1], self.gt_hist[i]
        if b[0] - a[0] > 2 * max_dt:
            return None
        r = (t - a[0]) / max(b[0] - a[0], 1e-9)
        return (t, a[1] + r * (b[1] - a[1]), a[2] + r * (b[2] - a[2]),
                a[3] + r * ang_diff(b[3], a[3]))

    def lookup(self, parent, child):
        try:
            tf = self.tf_buf.lookup_transform(parent, child, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        tr = tf.transform.translation
        t = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        return (t, tr.x, tr.y, yaw_of(tf.transform.rotation))

    def event(self, kind, text):
        t = self.rel_t(self.now_s())
        self.events.append((round(t, 2), kind, text))
        self.get_logger().info(f'[{kind}] t={t:7.1f}s  {text}')

    # ------------------------------------------------------------- callbacks
    def gt_cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.gt_t and t <= self.gt_t[-1]:
            return
        if self.t0 is None:
            self.t0 = t
        x, y = m.pose.position.x, m.pose.position.y
        if self.last_gt_xy is not None:
            d = math.hypot(x - self.last_gt_xy[0], y - self.last_gt_xy[1])
            if d < 1.0:   # buang lompatan aneh
                self.dist += d
        self.last_gt_xy = (x, y)
        self.gt_hist.append((t, x, y, yaw_of(m.pose.orientation)))
        self.gt_t.append(t)

    def info_cb(self, info):
        self.n_info += 1
        if info.ref_id > 0:
            self.node_stamp[info.ref_id] = info.header.stamp.sec + info.header.stamp.nanosec * 1e-9
        if info.loop_closure_id > 0:
            self.n_lc += 1
            self.event('LOOP', f'diterima: node {info.ref_id} -> {info.loop_closure_id}'
                               f'  (hyp={stat(info, "Loop/Highest_hypothesis_value/", 0):.2f},'
                               f' inliers={stat(info, "Loop/Visual_inliers/", -1):.0f})')
        if info.proximity_detection_id > 0:
            self.n_prox += 1
            self.event('PROX', f'diterima: node {info.ref_id} -> {info.proximity_detection_id}')

        hyp_id = stat(info, 'Loop/Highest_hypothesis_id/', 0) or 0
        rejected = stat(info, 'Loop/RejectedHypothesis/', 0) or 0
        if info.loop_closure_id <= 0 and hyp_id > 0:
            # ada hipotesis loop closure tapi tidak jadi diterima
            hv = stat(info, 'Loop/Highest_hypothesis_value/', 0) or 0
            if rejected > 0 or hv >= 0.11:   # 0.11 = Rtabmap/LoopThr default
                self.n_rej += 1
                err = stat(info, 'Loop/Optimization_max_error/m', 0) or 0
                inl = stat(info, 'Loop/Visual_inliers/', -1)
                why = []
                if inl is not None and inl >= 0:
                    why.append(f'inliers={inl:.0f}')
                if err > 0:
                    why.append(f'opt_err={err:.2f}m')
                self.event('TOLAK', f'kandidat node {info.ref_id} -> {int(hyp_id)} ditolak'
                                    f'  (hyp={hv:.2f} {" ".join(why)})')

    def graph_cb(self, m):
        self.map_graph = m

    def sample(self):
        if self.t0 is None:
            return
        slam = self.lookup('map', 'base_link')
        odom = self.lookup('odom', 'base_link')
        m2o = self.lookup('map', 'odom')
        tnow = self.gt_hist[-1][0]
        gt = self.gt_hist[-1]
        row = {'t': round(tnow - self.t0, 2), 'dist': round(self.dist, 3),
               'gt_x': gt[1], 'gt_y': gt[2], 'gt_yaw': gt[3]}
        for name, p in (('slam', slam), ('odom', odom)):
            if p is None:
                continue
            g = self.gt_at(p[0]) or gt
            row[f'{name}_x'], row[f'{name}_y'] = p[1], p[2]
            row[f'{name}_err'] = math.hypot(p[1] - g[1], p[2] - g[2])
            row[f'{name}_yaw_err_deg'] = math.degrees(ang_diff(p[3], g[3]))
        if m2o is not None:
            if self.last_m2o is not None:
                jump = math.hypot(m2o[1] - self.last_m2o[1], m2o[2] - self.last_m2o[2])
                jyaw = abs(math.degrees(ang_diff(m2o[3], self.last_m2o[3])))
                if jump > self.args.jump_thr or jyaw > 2.0:
                    self.max_jump = max(self.max_jump, jump)
                    self.event('KOREKSI', f'map->odom lompat {jump * 100:.1f} cm, {jyaw:.1f} deg')
            self.last_m2o = m2o
        row.update(n_lc=self.n_lc, n_prox=self.n_prox, n_rej=self.n_rej)
        self.rows.append(row)

        if tnow - self.last_print >= self.args.print_every:
            self.last_print = tnow
            se = row.get('slam_err', float('nan'))
            oe = row.get('odom_err', float('nan'))
            print(f't={row["t"]:7.1f}s  jarak={self.dist:6.2f}m  '
                  f'err SLAM={se * 100:6.1f}cm  err ODOM={oe * 100:6.1f}cm  '
                  f'loop={self.n_lc} prox={self.n_prox} tolak={self.n_rej}', flush=True)

        if self.args.duration > 0 and row['t'] >= self.args.duration:
            self.stop = True

    # ------------------------------------------------------------- laporan
    def graph_errors(self):
        """Error tiap node graf teroptimasi (/mapGraph) vs GT pada stamp node itu (/info)."""
        if self.map_graph is None:
            return []
        out = []
        for nid, pose in zip(self.map_graph.poses_id, self.map_graph.poses):
            t = self.node_stamp.get(nid)
            if t is None:
                continue
            g = self.gt_at(t, max_dt=0.5)
            if g is None:
                continue
            p = pose.position
            out.append((t - self.t0, p.x, p.y, g[1], g[2], math.hypot(p.x - g[1], p.y - g[2])))
        return out

    def report(self):
        out_dir = os.path.join(BASE_DIR, 'hasil_cek_drift', time.strftime('%Y%m%d_%H%M%S'))
        os.makedirs(out_dir, exist_ok=True)

        keys = ['t', 'dist', 'gt_x', 'gt_y', 'gt_yaw', 'slam_x', 'slam_y', 'slam_err',
                'slam_yaw_err_deg', 'odom_x', 'odom_y', 'odom_err', 'odom_yaw_err_deg',
                'n_lc', 'n_prox', 'n_rej']
        with open(os.path.join(out_dir, 'sampel.csv'), 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            for r in self.rows:
                w.writerow({k: r.get(k, '') for k in keys})
        with open(os.path.join(out_dir, 'event.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'jenis', 'keterangan'])
            w.writerows(self.events)
        gerr = self.graph_errors()
        with open(os.path.join(out_dir, 'error_graf.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'node_x', 'node_y', 'gt_x', 'gt_y', 'err'])
            w.writerows(gerr)

        def col(k):
            return [r[k] for r in self.rows if k in r]

        se, oe = col('slam_err'), col('odom_err')
        lines = ['=' * 70, 'RINGKASAN CEK DRIFT & LOOP CLOSURE', '=' * 70]
        dur = self.rows[-1]['t'] if self.rows else 0
        lines.append(f'Durasi (waktu sim)       : {dur:.1f} s')
        lines.append(f'Jarak tempuh (GT)        : {self.dist:.2f} m')
        lines.append(f'Pesan /rtabmap/info      : {self.n_info}')
        lines.append(f'Loop closure diterima    : {self.n_lc}')
        lines.append(f'Proximity diterima       : {self.n_prox}')
        lines.append(f'Kandidat ditolak         : {self.n_rej}')
        lines.append(f'Lompatan koreksi terbesar: {self.max_jump * 100:.1f} cm')
        if se:
            lines.append(f'Error pose SLAM  akhir/rata/maks: {se[-1]*100:.1f} / '
                         f'{sum(se)/len(se)*100:.1f} / {max(se)*100:.1f} cm')
        if oe:
            lines.append(f'Error pose ODOM  akhir/rata/maks: {oe[-1]*100:.1f} / '
                         f'{sum(oe)/len(oe)*100:.1f} / {max(oe)*100:.1f} cm')
        if oe and self.dist > 0.5:
            lines.append(f'Drift ODOM akhir         : {oe[-1] / self.dist * 100:.2f} % dari jarak')
        if gerr:
            ge = [g[5] for g in gerr]
            ge_sorted = sorted(ge)
            lines.append(f'Error node graf (peta) rata/median/maks: {sum(ge)/len(ge)*100:.1f} / '
                         f'{ge_sorted[len(ge)//2]*100:.1f} / {max(ge)*100:.1f} cm  ({len(ge)} node)')
            n_bad = sum(1 for e in ge if e > 0.10)
            lines.append(f'Node dengan error > 10 cm: {n_bad} ({n_bad / len(ge) * 100:.0f} %)')
        else:
            lines.append('Error node graf          : tidak tersedia (/mapGraph atau /info tidak diterima)')
        if self.map_graph is not None:
            types = [l.type for l in self.map_graph.links]
            lines.append(f'Link di graf akhir       : {len(self.map_graph.poses_id)} node, '
                         f'{types.count(1)} global loop closure, {types.count(2)} proximity (ruang), '
                         f'{types.count(3)} proximity (waktu)')

        lines.append('-' * 70)
        lines.append('DIAGNOSIS:')
        diag = []
        if self.n_info == 0:
            diag.append('- /rtabmap/info tidak pernah diterima. Apakah RTAB-Map jalan?')
        if self.dist > 5 and self.n_lc + self.n_prox == 0:
            diag.append('- Tidak ada satu pun loop closure/proximity yang diterima. '
                        'Drift odometri langsung masuk ke peta -> tembok dobel.')
        if self.n_rej > 0 and self.n_rej >= self.n_lc:
            diag.append(f'- {self.n_rej} kandidat loop closure DITOLAK (>= yang diterima). '
                        'Lihat event.csv: inliers kecil -> fitur visual kurang; '
                        'opt_err besar -> drift sudah melebihi RGBD/OptimizeMaxError.')
        if se and oe:
            ms, mo = sum(se) / len(se), sum(oe) / len(oe)
            if mo > 0.05 and ms > 0.8 * mo:
                diag.append(f'- Error SLAM ({ms*100:.1f} cm) hampir sama dengan ODOM ({mo*100:.1f} cm): '
                            'SLAM praktis tidak mengoreksi drift.')
            if ms > 1.2 * mo and ms > 0.05:
                diag.append(f'- Error SLAM ({ms*100:.1f} cm) LEBIH BESAR dari ODOM ({mo*100:.1f} cm): '
                            'kemungkinan ada loop closure SALAH. Cek event LOOP/KOREKSI yang besar.')
            if mo <= 0.05:
                diag.append('- Odometri sendiri sudah akurat (< 5 cm). Kalau peta tetap jelek, '
                            'penyebabnya kemungkinan di pembuatan grid (noise /scan, ray tracing), '
                            'bukan drift.')
        if gerr:
            ge = [g[5] for g in gerr]
            if max(ge) > 0.15:
                diag.append(f'- Sebagian node peta melenceng hingga {max(ge)*100:.0f} cm dari posisi '
                            'aslinya. Tembok dari node itu tergambar bergeser (sumber "tembok hantu").')
        if not diag:
            diag.append('- Tidak ada masalah besar yang terdeteksi.')
        lines += diag
        lines.append('-' * 70)
        lines.append(f'File hasil: {out_dir}')
        text = '\n'.join(lines)
        print('\n' + text)
        with open(os.path.join(out_dir, 'ringkasan.txt'), 'w') as f:
            f.write(text + '\n')
        self.plot(out_dir, gerr)

    def plot(self, out_dir, gerr):
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except ImportError:
            print('matplotlib tidak ada, grafik dilewati.')
            return
        if not self.rows:
            return
        fig, ax = plt.subplots(1, 2, figsize=(15, 6.5))
        a = ax[0]
        a.plot([g[1] for g in self.gt_hist], [g[2] for g in self.gt_hist], 'k-', lw=2,
               label='Ground truth')
        r = [x for x in self.rows if 'odom_x' in x]
        a.plot([x['odom_x'] for x in r], [x['odom_y'] for x in r], '-', color='tab:orange',
               lw=1, label='Odometri (EKF)')
        r = [x for x in self.rows if 'slam_x' in x]
        a.plot([x['slam_x'] for x in r], [x['slam_y'] for x in r], '-', color='tab:blue',
               lw=1, label='SLAM saat itu (map->base_link)')
        if gerr:
            a.plot([g[1] for g in gerr], [g[2] for g in gerr], '.', color='tab:green', ms=3,
                   label='Node graf teroptimasi (akhir)')
        a.set_aspect('equal')
        a.grid(alpha=0.3)
        a.set_xlabel('x (m)')
        a.set_ylabel('y (m)')
        a.set_title('Trajektori')
        a.legend(loc='best', fontsize=8)

        a = ax[1]
        r = [x for x in self.rows if 'slam_err' in x]
        a.plot([x['t'] for x in r], [x['slam_err'] * 100 for x in r], color='tab:blue',
               label='Error SLAM')
        r = [x for x in self.rows if 'odom_err' in x]
        a.plot([x['t'] for x in r], [x['odom_err'] * 100 for x in r], color='tab:orange',
               label='Error odometri')
        if gerr:
            a.plot([g[0] for g in gerr], [g[5] * 100 for g in gerr], '.', color='tab:green',
                   ms=3, label='Error node graf (akhir)')
        colors = {'LOOP': 'green', 'PROX': 'cyan', 'TOLAK': 'red', 'KOREKSI': 'purple'}
        used = set()
        for t, kind, _ in self.events:
            a.axvline(t, color=colors.get(kind, 'gray'), alpha=0.35, lw=1,
                      label=kind if kind not in used else None)
            used.add(kind)
        a.grid(alpha=0.3)
        a.set_xlabel('waktu sim (s)')
        a.set_ylabel('error posisi (cm)')
        a.set_title(f'Error vs waktu  (loop={self.n_lc}, prox={self.n_prox}, tolak={self.n_rej})')
        a.legend(loc='upper left', fontsize=8)
        fig.tight_layout()
        p = os.path.join(out_dir, 'grafik_drift.png')
        fig.savefig(p, dpi=120)
        print(f'Grafik: {p}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--duration', type=float, default=0, help='berhenti otomatis (detik sim), 0=Ctrl+C')
    ap.add_argument('--period', type=float, default=0.5, help='interval sampel (detik)')
    ap.add_argument('--print-every', type=float, default=5.0, help='interval cetak status (detik sim)')
    ap.add_argument('--info-topic', default='/info', help='topik rtabmap_msgs/Info')
    ap.add_argument('--graph-topic', default='/mapGraph', help='topik rtabmap_msgs/MapGraph')
    ap.add_argument('--jump-thr', type=float, default=0.20, help='ambang lompatan map->odom (m)')
    ap.add_argument('--wall-time', action='store_true', help='pakai jam dinding, bukan /clock')
    args = ap.parse_args()

    rclpy.init()
    node = CekDrift(args)
    print('Menunggu /ground_truth/pose dan TF... gerakkan robot. Ctrl+C untuk selesai.', flush=True)
    try:
        while rclpy.ok() and not node.stop:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    except Exception:  # konteks ROS sudah mati (Ctrl+C)
        pass
    finally:
        node.report()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
