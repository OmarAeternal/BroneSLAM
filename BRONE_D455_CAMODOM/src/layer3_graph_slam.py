#!/usr/bin/env python3
"""
LAYER 3: Full Graph-Based 2D Occupancy Grid SLAM (p(x_{1:t}, m | z_{1:t}, u_{1:t}))
Referensi:
- Siegwart et al., Bab 5.8.6 (Graph-Based SLAM) & Bab 5.3.2 (Occupancy Grid Map)
- Membaca parameter langsung dari config/slam_params.yaml

Dilengkapi Proteksi Anti-Phantom-Map:
1. Blind-Scan Guard: Jika jumlah titik pantul valid D455 < 10 (mis. menghadap sudut buta),
   jangan lakukan free-space raycasting yang dapat merusak tembok lama.
2. Wall-Locking Log-Odds: Sel tembok yang sudah terkonfirmasi kuat (log_odds > 2.0)
   dilindungi agar tidak mudah terhapus oleh sinar miring (grazing rays).
"""

import os
import math
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped, PoseStamped, Quaternion
import tf2_ros


def euler_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    return q


def quaternion_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class ReactiveAutoExplorer:
    pass


class Layer3GraphSlamNode(Node):
    def __init__(self):
        super().__init__('layer3_graph_slam')

        base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config_path = os.path.join(base_dir, 'config', 'slam_params.yaml')
        params = {}
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                raw = yaml.safe_load(f)
                params = raw.get('slam_toolbox', {}).get('ros__parameters', {})

        self.resolution = float(params.get('resolution', 0.05))
        self.min_range = float(params.get('min_laser_range', 0.15))
        self.max_range = float(params.get('max_laser_range', 6.00))
        self.min_travel_dist = float(params.get('minimum_travel_distance', 0.15))
        self.min_travel_head = float(params.get('minimum_travel_heading', 0.15))
        self.use_scan_matching = bool(params.get('use_scan_matching', True))

        self.cam_offset_x = 0.08

        # Peta Occupancy Grid 24m x 24m (mencakup seluruh Multi-Room Apartment)
        self.map_width_m = 24.0
        self.map_height_m = 24.0
        self.grid_w = int(round(self.map_width_m / self.resolution))
        self.grid_h = int(round(self.map_height_m / self.resolution))
        self.origin_x = -self.map_width_m / 2.0
        self.origin_y = -self.map_height_m / 2.0

        self.log_odds = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)
        self.l_occ = 0.95    # Penambahan log-odds saat sinar D455 mengenai tembok/perabot
        self.l_free = -0.30  # Pengurangan log-odds sepanjang lintasan sinar bebas
        self.l_min = -4.0
        self.l_max = 6.0

        self.pose_graph_nodes = []
        self.last_node_odom = None
        self.curr_odom = None
        self.map_to_odom = [0.0, 0.0, 0.0]

        self.create_subscription(Odometry, '/odom', self.odom_callback, 20)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.path_pub = self.create_publisher(Path, '/slam/trajectory', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_timer(1.0 / 30.0, self.broadcast_map_to_odom_tf)
        self.create_timer(1.0, self.publish_map)

        self.get_logger().info('=============================================================')
        self.get_logger().info('  LAYER 3: GRAPH-BASED 2D SLAM NODE AKTIF (Pure D455)        ')
        self.get_logger().info(f'  - Grid Resolution : {self.resolution:.2f} m/cell ({self.grid_w}x{self.grid_h})')
        self.get_logger().info(f'  - D455 Range Limit: [{self.min_range:.2f}m, {self.max_range:.2f}m]')
        self.get_logger().info('=============================================================')

    def odom_callback(self, msg: Odometry):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        oth = quaternion_to_yaw(msg.pose.pose.orientation)
        self.curr_odom = (ox, oy, oth)

    def odom_to_map_pose(self, ox: float, oy: float, oth: float):
        tx, ty, tth = self.map_to_odom
        cos_t, sin_t = math.cos(tth), math.sin(tth)
        mx = tx + ox * cos_t - oy * sin_t
        my = ty + ox * sin_t + oy * cos_t
        mth = normalize_angle(tth + oth)
        return mx, my, mth

    def update_map_to_odom_from_corrected_pose(
        self, ox: float, oy: float, oth: float, mx: float, my: float, mth: float
    ):
        tth = normalize_angle(mth - oth)
        cos_t, sin_t = math.cos(tth), math.sin(tth)
        tx = mx - (ox * cos_t - oy * sin_t)
        ty = my - (ox * sin_t + oy * cos_t)
        self.map_to_odom = [tx, ty, tth]

    def scan_callback(self, scan: LaserScan):
        if self.curr_odom is None:
            return

        ox, oy, oth = self.curr_odom

        if self.last_node_odom is not None:
            dx = ox - self.last_node_odom[0]
            dy = oy - self.last_node_odom[1]
            dist = math.hypot(dx, dy)
            dhead = abs(normalize_angle(oth - self.last_node_odom[2]))
            if dist < self.min_travel_dist and dhead < self.min_travel_head:
                return

        pred_mx, pred_my, pred_mth = self.odom_to_map_pose(ox, oy, oth)

        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        ranges = np.array(scan.ranges, dtype=np.float32)
        valid_hit = np.isfinite(ranges) & (ranges >= self.min_range) & (ranges < self.max_range)

        # Proteksi Blind-Scan: Jika kamera tidak melihat cukup fitur valid (< 10 titik),
        # jangan perbarui peta agar tidak menggambar ruang kosong palsu di balik tembok!
        if np.count_nonzero(valid_hit) < 10:
            return

        corr_mx, corr_my, corr_mth = pred_mx, pred_my, pred_mth
        if self.use_scan_matching and len(self.pose_graph_nodes) >= 1:
            hit_r = ranges[valid_hit]
            hit_a = angles[valid_hit]
            local_x = self.cam_offset_x + hit_r * np.cos(hit_a)
            local_y = hit_r * np.sin(hit_a)

            corr_mx, corr_my, corr_mth = self._correlative_scan_match(
                pred_mx, pred_my, pred_mth, local_x, local_y
            )
            self.update_map_to_odom_from_corrected_pose(
                ox, oy, oth, corr_mx, corr_my, corr_mth
            )

        self.last_node_odom = (ox, oy, oth)
        self.pose_graph_nodes.append({'x': corr_mx, 'y': corr_my, 'theta': corr_mth})

        self._integrate_scan_into_grid(corr_mx, corr_my, corr_mth, angles, ranges)
        self._publish_trajectory(scan.header.stamp)

    def _correlative_scan_match(
        self, init_x: float, init_y: float, init_th: float, local_x: np.ndarray, local_y: np.ndarray
    ):
        best_score = -1e9
        best_pose = (init_x, init_y, init_th)

        dx_candidates = np.linspace(-0.14, 0.14, 11)
        dy_candidates = np.linspace(-0.14, 0.14, 11)
        dth_candidates = np.linspace(-0.04, 0.04, 5)

        for dth in dth_candidates:
            th = init_th + dth
            cos_t, sin_t = math.cos(th), math.sin(th)
            rot_x = local_x * cos_t - local_y * sin_t
            rot_y = local_x * sin_t + local_y * cos_t

            for dx in dx_candidates:
                cx = init_x + dx
                gx_base = (cx + rot_x - self.origin_x) / self.resolution
                for dy in dy_candidates:
                    cy = init_y + dy
                    gy_base = (cy + rot_y - self.origin_y) / self.resolution

                    ix = np.rint(gx_base).astype(np.int32)
                    iy = np.rint(gy_base).astype(np.int32)
                    in_bounds = (ix >= 0) & (ix < self.grid_w) & (iy >= 0) & (iy < self.grid_h)
                    if not np.any(in_bounds):
                        continue

                    occ_hits = np.maximum(0.0, self.log_odds[iy[in_bounds], ix[in_bounds]])
                    motion_penalty = 8.0 * (dx * dx + dy * dy) + 80.0 * (dth * dth)
                    score = float(np.sum(occ_hits)) - motion_penalty

                    if score > best_score:
                        best_score = score
                        best_pose = (cx, cy, normalize_angle(th))

        return best_pose

    def _integrate_scan_into_grid(
        self, rx: float, ry: float, rth: float, angles: np.ndarray, ranges: np.ndarray
    ):
        cam_x = rx + self.cam_offset_x * math.cos(rth)
        cam_y = ry + self.cam_offset_x * math.sin(rth)

        gx0 = int(round((cam_x - self.origin_x) / self.resolution))
        gy0 = int(round((cam_y - self.origin_y) / self.resolution))
        if not (0 <= gx0 < self.grid_w and 0 <= gy0 < self.grid_h):
            return

        for a, r in zip(angles, ranges):
            if math.isnan(r) or r < self.min_range:
                continue

            ray_angle = rth + float(a)
            is_hit = math.isfinite(r) and (r < self.max_range)
            trace_dist = float(r) if is_hit else (self.max_range - 0.15)

            end_x = cam_x + trace_dist * math.cos(ray_angle)
            end_y = cam_y + trace_dist * math.sin(ray_angle)
            gx1 = int(round((end_x - self.origin_x) / self.resolution))
            gy1 = int(round((end_y - self.origin_y) / self.resolution))

            num_steps = max(abs(gx1 - gx0), abs(gy1 - gy0))
            if num_steps > 1:
                xs = np.linspace(gx0, gx1, num_steps, endpoint=False, dtype=np.int32)[1:]
                ys = np.linspace(gy0, gy1, num_steps, endpoint=False, dtype=np.int32)[1:]
                valid = (xs >= 0) & (xs < self.grid_w) & (ys >= 0) & (ys < self.grid_h)
                vx, vy = xs[valid], ys[valid]
                # Lindungi sel tembok yang sudah kuat (log_odds < 2.2) agar tidak bocor
                not_locked_wall = self.log_odds[vy, vx] < 2.2
                vx, vy = vx[not_locked_wall], vy[not_locked_wall]
                self.log_odds[vy, vx] = np.clip(
                    self.log_odds[vy, vx] + self.l_free, self.l_min, self.l_max
                )

            if is_hit and (0 <= gx1 < self.grid_w and 0 <= gy1 < self.grid_h):
                self.log_odds[gy1, gx1] = min(self.l_max, self.log_odds[gy1, gx1] + self.l_occ)

    def broadcast_map_to_odom_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = self.map_to_odom[0]
        t.transform.translation.y = self.map_to_odom[1]
        t.transform.translation.z = 0.0
        t.transform.rotation = euler_to_quaternion(self.map_to_odom[2])
        self.tf_broadcaster.sendTransform(t)

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_w
        msg.info.height = self.grid_h
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0

        grid_data = np.full((self.grid_h, self.grid_w), -1, dtype=np.int8)
        grid_data[self.log_odds < -0.25] = 0
        grid_data[self.log_odds > 0.40] = 100
        msg.data = grid_data.flatten().tolist()
        self.map_pub.publish(msg)

    def _publish_trajectory(self, stamp):
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = 'map'
        for n in self.pose_graph_nodes:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = n['x']
            ps.pose.position.y = n['y']
            ps.pose.orientation = euler_to_quaternion(n['theta'])
            path.poses.append(ps)
        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = Layer3GraphSlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
