#!/usr/bin/env python3
"""
Auto-Explorer BRONE (ROS 2) -- robot jalan sendiri memetakan ruangan, berbasis peta + lokalisasi.

Mode:
  frontier (default) : eksplorasi sistematis ke batas area yang belum terpetakan (frontier)
                       sampai tidak ada frontier lagi -> peta lengkap -> kembali ke titik awal.
  random             : jalan ACAK ke titik-titik acak di area yang sudah terpetakan (terus-menerus).
  goto               : pergi ke kotak grid ruangan (a-h x 1-8, lihat peta_grid/peta_grid.png),
                       cukup tulis kotaknya: --2c atau --8h (boleh beberapa: --2c --8h -> berurutan).

Anti-tabrak (kedua mode):
  * Lokalisasi = TF map->base_link dari RTAB-Map (+ EKF odom->base_link).
  * Jarak TEPI robot ke tembok dihitung dari peta /map (distance field) DAN memori titik /scan
    5 detik terakhir (untuk halangan yang sudah masuk zona buta D455 < 0.293 m).
  * Setiap perintah diprediksi 1 s ke depan; bila jarak tepi robot < min_clearance (5 cm)
    perintah diperlambat / ditolak. Perencana jalur memakai min_clearance + pose_margin.
  * TF/peta hilang -> robot berhenti.

Cara pakai (suite RTAB-Map sudah jalan, JANGAN jalankan teleop_explorer.py bersamaan):
  python3 src/auto_explorer.py                 # mode frontier
  python3 src/auto_explorer.py --mode random   # mode jalan acak
  python3 src/auto_explorer.py --mode random --min-clearance 0.05 --max-speed 0.18
  python3 src/auto_explorer.py --2c            # pergi ke kotak 2c lalu berhenti
  python3 src/auto_explorer.py --2c --8h       # ke 2c, lalu ke 8h
  python3 src/auto_explorer.py --grid          # tampilkan grid & kotak posisi start

Topik: sub /map (OccupancyGrid), /scan; pub /cmd_vel, /explore/path, /explore/goal
"""

import argparse
import math
import os
import re
import sys
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.clock import Clock
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
import tf2_ros

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from explore_core import Explorer, ExplorerConfig, GridMap  # noqa: E402
import room_grid  # noqa: E402


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class AutoExplorerNode(Node):
    def __init__(self, cfg: ExplorerConfig, seed, duration, sim_time, extra_xy=None):
        super().__init__('auto_explorer',
                         parameter_overrides=[Parameter('use_sim_time', value=sim_time)])
        self.cfg = cfg
        self.duration = duration
        self.extra_xy = extra_xy or []   # sudut kotak tujuan goto (frame map) -> grid diperluas
        rng = np.random.default_rng(None if seed < 0 else seed)
        self.explorer = Explorer(cfg, rng=rng, log=self.get_logger().info)

        self.tf_buf = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

        map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/explore/path', 1)
        self.goal_pub = self.create_publisher(PoseStamped, '/explore/goal', 1)

        self.mem = deque()            # (t_wall, titik (N,2) di frame odom)
        self.mem_s = 5.0
        self.map_wall = None
        self.t_start = None
        self.last_info = 0.0
        self.no_pose_since = None
        self.last_path_id = None
        self.done_reported = False

        # Timer berbasis jam dinding: tetap berhenti aman walau /clock macet
        self.timer = self.create_timer(0.1, self.control, clock=Clock())
        self.get_logger().info('=' * 64)
        self.get_logger().info(f'  AUTO-EXPLORER  mode={cfg.mode.upper()}  radius robot={cfg.robot_radius:.2f} m')
        self.get_logger().info(f'  jarak min tepi robot->tembok={cfg.min_clearance*100:.0f} cm '
                               f'(+{cfg.pose_margin*100:.0f} cm cadangan lokalisasi di perencana)')
        self.get_logger().info(f'  kecepatan maks {cfg.max_v:.2f} m/s, putar {cfg.max_w:.2f} rad/s')
        self.get_logger().info('  Menunggu /map dan TF map->base_link ...')
        self.get_logger().info('=' * 64)

    # ------------------------------------------------------------------ input
    def map_cb(self, msg: OccupancyGrid):
        info = msg.info
        if info.width == 0 or info.height == 0:
            return
        gm = GridMap(msg.data, info.width, info.height, info.resolution,
                     info.origin.position.x, info.origin.position.y, extra_xy=self.extra_xy)
        self.explorer.update_map(gm)
        self.map_wall = time.monotonic()

    def scan_cb(self, msg: LaserScan):
        try:
            tf = self.tf_buf.lookup_transform('odom', msg.header.frame_id, Time.from_msg(msg.header.stamp),
                                              timeout=Duration(seconds=0.0))
        except Exception:
            try:
                tf = self.tf_buf.lookup_transform('odom', msg.header.frame_id, Time())
            except Exception:
                return
        r = np.asarray(msg.ranges, dtype=np.float32)
        a = msg.angle_min + np.arange(r.size) * msg.angle_increment
        ok = np.isfinite(r) & (r >= msg.range_min) & (r <= min(msg.range_max, 3.0))
        if not ok.any():
            return
        px, py = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        tr = tf.transform.translation
        yaw = yaw_of(tf.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        pts = np.column_stack((tr.x + c * px - s * py, tr.y + s * px + c * py))
        pts = np.unique(np.round(pts / 0.02) * 0.02, axis=0)   # voxel 2 cm
        now = time.monotonic()
        self.mem.append((now, pts))
        while self.mem and now - self.mem[0][0] > self.mem_s:
            self.mem.popleft()

    def _pose(self, target, src='base_link'):
        tf = self.tf_buf.lookup_transform(target, src, Time())
        t = tf.transform
        return t.translation.x, t.translation.y, yaw_of(t.rotation)

    def _mem_in_base(self, odom_pose):
        if not self.mem:
            return np.zeros((0, 2))
        pts = np.concatenate([m[1] for m in self.mem])
        x, y, th = odom_pose
        c, s = math.cos(th), math.sin(th)
        dx, dy = pts[:, 0] - x, pts[:, 1] - y
        pb = np.column_stack((c * dx + s * dy, -s * dx + c * dy))
        return pb[np.hypot(pb[:, 0], pb[:, 1]) < 1.5]

    # ------------------------------------------------------------------ loop
    def stop(self):
        self.cmd_pub.publish(Twist())

    def control(self):
        now = time.monotonic()
        try:
            pose = self._pose('map')
            odom_pose = self._pose('odom')
            self.no_pose_since = None
        except Exception:
            if self.no_pose_since is None:
                self.no_pose_since = now
                self.get_logger().warn('TF map->base_link belum ada / hilang -> robot berhenti')
            self.stop()
            return
        if self.map_wall is None or now - self.map_wall > 10.0:
            if self.map_wall is not None and now - self.last_info > 3.0:
                self.get_logger().warn('/map tidak diperbarui > 10 s -> robot berhenti')
                self.last_info = now
            self.stop()
            return
        # Waktu state machine = waktu ROS (/clock simulasi), karena gerak robot ikut waktu sim
        t_ros = self.get_clock().now().nanoseconds * 1e-9
        if self.t_start is None:
            self.t_start = t_ros
        t = t_ros - self.t_start
        if self.duration > 0 and t > self.duration and self.explorer.state != 'DONE':
            self.get_logger().info(f'Durasi {self.duration:.0f} s tercapai -> berhenti.')
            self.explorer.state = 'DONE'

        ex = self.explorer
        vx, vy, wz = ex.step(t, pose, self._mem_in_base(odom_pose))
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = float(vx), float(vy), float(wz)
        self.cmd_pub.publish(cmd)

        if ex.path is not None and id(ex.path) != self.last_path_id:
            self.last_path_id = id(ex.path)
            self._publish_path(ex.path, ex.goal)

        if now - self.last_info > 5.0:
            self.last_info = now
            gm = ex.gm
            self.get_logger().info(
                f'[{ex.cfg.mode}] state={ex.state:9s} tujuan selesai={ex.goals_done:3d} '
                f'luas terpetakan={gm.known_area_m2:6.1f} m2  jarak tembok sekarang='
                f'{ex.safety.clearance_now(gm, pose, None)*100:5.1f} cm  min={ex.min_clear_seen*100:5.1f} cm  '
                f'filter={ex.last_status}')
        if ex.state == 'DONE' and not self.done_reported:
            self.done_reported = True
            self.get_logger().info('=' * 64)
            self.get_logger().info(f'  SELESAI. Luas terpetakan {ex.gm.known_area_m2:.1f} m2, '
                                   f'{ex.goals_done} tujuan, jarak tembok terdekat {ex.min_clear_seen*100:.1f} cm')
            self.get_logger().info('  Peta tersimpan otomatis oleh RTAB-Map di ~/.ros/rtabmap.db')
            self.get_logger().info('=' * 64)

    def _publish_path(self, pts, goal):
        stamp = self.get_clock().now().to_msg()
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = stamp
        for x, y in pts:
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x, p.pose.position.y = float(x), float(y)
            p.pose.orientation.w = 1.0
            path.poses.append(p)
        self.path_pub.publish(path)
        if goal is not None:
            g = PoseStamped()
            g.header = path.header
            g.pose.position.x, g.pose.position.y = float(goal[0]), float(goal[1])
            g.pose.orientation.w = 1.0
            self.goal_pub.publish(g)


def main():
    ap = argparse.ArgumentParser(description='BRONE auto-explorer (frontier / random)')
    ap.add_argument('--mode', choices=['frontier', 'random', 'goto'], default=None,
                    help='default frontier; otomatis goto bila ada kotak (--2c)')
    ap.add_argument('--goto', default='', help='kotak tujuan dipisah koma, mis. 2c,8h (sama dengan --2c --8h)')
    ap.add_argument('--start-world', type=float, nargs=3, metavar=('X', 'Y', 'YAW'), default=None,
                    help='pose start robot di dunia Webots (default: dibaca dari file world)')
    ap.add_argument('--grid', action='store_true', help='tampilkan grid ruangan lalu keluar')
    ap.add_argument('--min-clearance', type=float, default=0.05, help='jarak min tepi robot ke tembok (m)')
    ap.add_argument('--pose-margin', type=float, default=0.03, help='cadangan error lokalisasi (m)')
    ap.add_argument('--robot-radius', type=float, default=0.13)
    ap.add_argument('--max-speed', type=float, default=0.22)
    ap.add_argument('--no-spin', action='store_true', help='frontier: jangan putar 360 deg di tiap tujuan')
    ap.add_argument('--no-return', action='store_true', help='frontier: jangan kembali ke titik awal')
    ap.add_argument('--duration', type=float, default=0.0, help='batas waktu (s), 0 = tanpa batas')
    ap.add_argument('--seed', type=int, default=-1, help='seed acak (mode random)')
    ap.add_argument('--wall-time', action='store_true', help='pakai jam dinding (bukan /clock)')
    # Kotak grid ditulis langsung sebagai opsi: --2c, --8h, --c2 ...
    argv, cells = [], []
    for a in sys.argv[1:]:
        if a.startswith('-') and room_grid.is_cell_token(a):
            cells.append(room_grid.parse_cell(a))
        elif re.match(r'^--?(\d+[a-zA-Z]|[a-zA-Z]\d+)$', a):
            sys.exit(f'Kotak {a} tidak ada. Grid: kolom a-h, baris 1-8 (contoh --2c, --8h). Lihat --grid')
        else:
            argv.append(a)
    args, ros_args = ap.parse_known_args(argv)
    cells += [room_grid.parse_cell(c) for c in args.goto.split(',') if c.strip()]

    tfw = room_grid.WorldMapTransform(tuple(args.start_world) if args.start_world else None)
    start_cell = room_grid.world_to_cell(tfw.x0, tfw.y0)
    if args.grid:
        print(f'Grid ruangan {room_grid.N}x{room_grid.N}: kolom a-h kiri->kanan, baris 1-8 atas->bawah, '
              f'kotak {room_grid.CELL_W:.2f} x {room_grid.CELL_H:.2f} m. "--" = di luar apartemen.')
        for r in range(1, room_grid.N + 1):
            row = []
            for c in room_grid.COLS:
                lab = f'{r}{c}'
                row.append(('[' + lab + ']') if lab == start_cell else
                           (' ' + lab + ' ' if room_grid.cell_in_room(lab) else ' -- '))
            print(' '.join(row))
        print(f'[..] = posisi start robot ({tfw.x0:.2f}, {tfw.y0:.2f}). Gambar: peta_grid/peta_grid.png')
        return
    mode = args.mode or ('goto' if cells else 'frontier')
    if mode == 'goto' and not cells:
        ap.error('mode goto butuh kotak tujuan, mis. --2c')
    if cells and mode != 'goto':
        ap.error('kotak tujuan (--2c) hanya untuk mode goto')
    targets = [tfw.cell_target(c) for c in cells]
    for c in cells:
        if not room_grid.cell_in_room(c):
            print(f'[PERINGATAN] kotak {c.upper()} di luar apartemen -> kemungkinan tidak terjangkau')
    extra = [p for tg in targets for p in tg['poly']]

    cfg = ExplorerConfig(mode=mode, targets=targets, min_clearance=args.min_clearance, pose_margin=args.pose_margin,
                         robot_radius=args.robot_radius, max_v=args.max_speed,
                         spin_at_goal=not args.no_spin, return_home=not args.no_return)
    # Signal handler rclpy dimatikan: Ctrl+C harus sempat mengirim STOP sebelum shutdown,
    # karena driver menahan /cmd_vel terakhir (robot bisa terus jalan).
    rclpy.init(args=[sys.argv[0]] + ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = AutoExplorerNode(cfg, args.seed, args.duration, sim_time=not args.wall_time, extra_xy=extra)
    if cells:
        node.get_logger().info(f"  Start di kotak {(start_cell or '?').upper()} -> tujuan: "
                               f"{' -> '.join(c.upper() for c in cells)}")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.get_logger().info('Berhenti -> kirim STOP ke /cmd_vel')
            node.stop()
            time.sleep(0.1)
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
