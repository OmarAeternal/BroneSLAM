#!/usr/bin/env python3
"""
LAYER 1.5A: RGB-D 3D Visual Odometry Node (Bab 5.8.5 Visual SLAM / VIO)
Referensi: Siegwart et al., Bab 5.8.5 (Visual Odometry & 3D Feature Tracking)

Tugas Node Ini:
1. Mendeteksi titik-titik fitur sudut (Visual Keypoints - Shi-Tomasi / FAST)
   pada citra warna RealSense D455 (/camera/color/image_raw).
2. Melacak perpindahan titik fitur antar-frame menggunakan Pyramidal Lucas-Kanade
   Optical Flow dan memproyeksikannya ke koordinat 3D (X, Y, Z) menggunakan
   citra kedalaman (/camera/depth/image_raw) serta matriks intrinsik K (/camera/camera_info).
3. Menghitung kecepatan translasi kamera (vx_vo, vy_vo) secara robuste (Median / Trimmed Estimator)
   dan menentukan Matriks Kovariansi Dinamis R_vo berdasarkan jumlah titik fitur valid:
   - Banyak fitur visual (>= 20 titik) -> R_vo KECIL (0.002) -> EKF percaya penuh pada Kamera!
   - Tembok polos (< 8 titik)          -> R_vo BESAR (50.0)  -> EKF otomatis beralih ke Roda + IMU!
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


# Posisi camera_link di base_link (harus sama dengan TF statis di d455_rtabmap_driver.py)
CAM_X_IN_BASE = 0.08
CAM_Y_IN_BASE = 0.0


def _kabsch_2d(a: np.ndarray, b: np.ndarray):
    """R, t (least squares) sehingga b ~= R a + t. a, b: (N, 2)."""
    ca, cb = a.mean(axis=0), b.mean(axis=0)
    h = (a - ca).T @ (b - cb)
    th = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
    c, s = math.cos(th), math.sin(th)
    r = np.array([[c, -s], [s, c]])
    return th, r, cb - r @ ca


def estimate_rigid_2d_ransac(a: np.ndarray, b: np.ndarray, rng, iters: int = 60):
    """
    Estimasi gerak planar robot dari pasangan titik (a = frame sekarang, b = frame sebelumnya).
    Return (dtheta, t[2], inlier_mask, rms) atau None. Ambang inlier naik dengan jarak
    karena noise depth D455 ~ kuadrat jarak.
    """
    n = len(a)
    if n < 3:
        return None
    rng_dist = np.linalg.norm(b, axis=1)
    thr = 0.02 + 0.012 * rng_dist * rng_dist
    best = None
    best_cnt = 0
    for _ in range(iters):
        i, j = rng.choice(n, 2, replace=False)
        if np.linalg.norm(a[i] - a[j]) < 0.15:
            continue
        _, r, t = _kabsch_2d(a[[i, j]], b[[i, j]])
        res = np.linalg.norm(a @ r.T + t - b, axis=1)
        inl = res < thr
        cnt = int(np.count_nonzero(inl))
        if cnt > best_cnt:
            best_cnt, best = cnt, inl
    if best is None or best_cnt < 3:
        return None
    th, r, t = _kabsch_2d(a[best], b[best])
    res = np.linalg.norm(a @ r.T + t - b, axis=1)
    inl = res < thr
    if np.count_nonzero(inl) >= 3:
        th, r, t = _kabsch_2d(a[inl], b[inl])
        res = np.linalg.norm(a @ r.T + t - b, axis=1)
    rms = float(np.sqrt(np.mean(res[inl] ** 2))) if np.any(inl) else 1.0
    return th, t, inl, rms


class RgbdVisualOdometryNode(Node):
    def __init__(self):
        super().__init__('rgbd_visual_odometry')

        # fallback = fx color D455 asli 640x480 (ditimpa camera_info saat runtime)
        self.fx = 379.907
        self.fy = 379.907
        self.cx = 319.5
        self.cy = 239.5

        self.latest_depth = None
        self.prev_gray = None
        self.prev_pts = None
        self.prev_pts_3d = None
        self.prev_time = None
        self.rng = np.random.default_rng(0)

        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.info_callback, 10)
        self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, 10)

        self.vo_pub = self.create_publisher(Odometry, '/vo/odom', 10)
        self.get_logger().info('=============================================================')
        self.get_logger().info('  LAYER 1.5A: RGB-D 3D VISUAL ODOMETRY NODE AKTIF            ')
        self.get_logger().info('  - Melacak fitur 3D (u,v,Z) -> /vo/odom + Dynamic Covariance')
        self.get_logger().info('=============================================================')

    def info_callback(self, msg: CameraInfo):
        if msg.k[0] > 0:
            self.fx = float(msg.k[0])
            self.fy = float(msg.k[4])
            self.cx = float(msg.k[2])
            self.cy = float(msg.k[5])

    def depth_callback(self, msg: Image):
        self.latest_depth = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))

    def _pixels_to_3d_robot_frame(self, pts_2d: np.ndarray, depth_img: np.ndarray):
        """
        Memproyeksikan piksel 2D (u, v) + Depth Z menjadi titik 3D di kerangka bodi robot:
        X_rob (maju) = Z_opt
        Y_rob (kiri) = -(u - cx) * Z_opt / fx
        """
        h, w = depth_img.shape
        u = np.clip(np.rint(pts_2d[:, 0]).astype(np.int32), 0, w - 1)
        v = np.clip(np.rint(pts_2d[:, 1]).astype(np.int32), 0, h - 1)
        z = depth_img[v, u]
        valid = np.isfinite(z) & (z >= 0.293) & (z < 5.5)   # D455 asli buta < 0.293 m

        x_rob = z
        y_rob = -(pts_2d[:, 0] - self.cx) * z / self.fx
        pts_3d = np.stack([x_rob, y_rob, z], axis=1)
        return pts_3d, valid

    def rgb_callback(self, msg: Image):
        if self.latest_depth is None:
            return

        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))

        if HAS_CV2:
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        else:
            gray = np.mean(bgr, axis=2).astype(np.uint8)

        vx_vo, vy_vo, wz_vo = 0.0, 0.0, 0.0
        num_inliers = 0
        sigma_vo_sq = 50.0  # Default ketidakpastian tinggi jika fitur sedikit

        if HAS_CV2 and self.prev_gray is not None and self.prev_pts is not None and self.prev_time is not None:
            dt = now_sec - self.prev_time
            if 0.005 < dt < 0.5 and len(self.prev_pts) >= 8:
                next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                    self.prev_gray, gray, self.prev_pts, None,
                    winSize=(15, 15), maxLevel=2
                )
                if next_pts is not None and status is not None:
                    good_mask = (status.flatten() == 1)
                    if np.count_nonzero(good_mask) >= 8:
                        p_prev_3d = self.prev_pts_3d[good_mask]
                        p_curr_2d = next_pts.reshape(-1, 2)[good_mask]
                        p_curr_3d, valid_curr = self._pixels_to_3d_robot_frame(p_curr_2d, self.latest_depth)

                        if np.count_nonzero(valid_curr) >= 8:
                            # Titik kamera -> frame base_link (kamera 8 cm di depan pusat robot).
                            # Gerak robot = transformasi rigid 2D (rotasi + translasi) yang memetakan
                            # titik frame sekarang ke frame sebelumnya: p_prev = R * p_curr + t.
                            # (Versi lama hanya memakai selisih titik -> rotasi murni terbaca sebagai
                            #  geser samping palsu sebesar w*Z, lalu EKF ikut "meluncur".)
                            off = np.array([CAM_X_IN_BASE, CAM_Y_IN_BASE])
                            a = p_curr_3d[valid_curr, :2] + off
                            b = p_prev_3d[valid_curr, :2] + off
                            est = estimate_rigid_2d_ransac(a, b, self.rng)
                            if est is not None:
                                dth, t, inl, rms = est
                                num_inliers = int(np.count_nonzero(inl))
                                v = t / dt
                                if num_inliers >= 8 and np.hypot(v[0], v[1]) < 1.5 \
                                        and abs(dth / dt) < 3.0:
                                    vx_vo, vy_vo, wz_vo = float(v[0]), float(v[1]), dth / dt
                                    # Variansi kecepatan dari residu: std posisi ~ rms/sqrt(n)
                                    sd_v = 2.0 * rms / math.sqrt(num_inliers) / dt
                                    sigma_vo_sq = max(0.01, sd_v * sd_v)

        # Deteksi fitur sudut baru untuk frame berikutnya
        if HAS_CV2:
            corners = cv2.goodFeaturesToTrack(gray, maxCorners=120, qualityLevel=0.02, minDistance=12)
            if corners is not None and len(corners) > 0:
                pts_2d = corners.reshape(-1, 2)
                pts_3d, valid_3d = self._pixels_to_3d_robot_frame(pts_2d, self.latest_depth)
                if np.count_nonzero(valid_3d) > 0:
                    self.prev_pts = pts_2d[valid_3d].reshape(-1, 1, 2).astype(np.float32)
                    self.prev_pts_3d = pts_3d[valid_3d]
                else:
                    self.prev_pts = None
            else:
                self.prev_pts = None

        self.prev_gray = gray
        self.prev_time = now_sec

        # Publish /vo/odom beserta Matriks Kovariansi Dinamis R_vo untuk EKF
        vo_msg = Odometry()
        vo_msg.header.stamp = msg.header.stamp
        vo_msg.header.frame_id = 'odom'
        vo_msg.child_frame_id = 'base_link'
        vo_msg.twist.twist.linear.x = vx_vo
        vo_msg.twist.twist.linear.y = vy_vo
        vo_msg.twist.twist.angular.z = wz_vo       # informasi saja (EKF memakai gyro untuk yaw)
        vo_msg.twist.covariance[0] = sigma_vo_sq   # Variansi vx_vo (dinamis sesuai residu)
        vo_msg.twist.covariance[7] = sigma_vo_sq   # Variansi vy_vo
        vo_msg.twist.covariance[35] = sigma_vo_sq
        self.vo_pub.publish(vo_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RgbdVisualOdometryNode()
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
