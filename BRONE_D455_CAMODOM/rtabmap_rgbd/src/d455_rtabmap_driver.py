#!/usr/bin/env python3
"""
RTAB-Map RGB-D + EKF Driver Node -- VERSI SPEK D455 ASLI (BRONE_D455_REAL)
=============================================================================
Perbedaan dengan BRONE_RESEARCH (lihat d455_real_model.py untuk sumber angka):
  * FOV/fx depth 79.16 deg / 387.07 px, color 80.21 deg / 379.907 px
  * Color terpisah 59 mm dari depth; depth di-ALIGN ke color
    -> /camera/aligned_depth_to_color/image_raw + /camera/color/camera_info
  * Depth: noise ~Z^2, kuantisasi 1 mm, min 0.293 m, bayangan stereo, dropout
  * IMU: hanya gyro (bias+noise) + accel; TIDAK ada orientasi absolut.
    Yaw odometri roda = integrasi gyro (ikut drift seperti kamera asli)
  * Laju data mengikuti profil D455_PROFILE=usb3 (default) / usb2
  * D455_REALISM=0 -> matikan noise/artefak (intrinsik tetap asli)
  * D455_BRAKE_DIST (default 0.40 m): rem anti-tabrak; nilai lama 0.28 m
    tidak akan pernah aktif karena kamera asli buta < 0.293 m

Topik:
  /camera/color/image_raw (bgr8)            /camera/color/camera_info
  /camera/aligned_depth_to_color/image_raw  (32FC1, m, 0 = kosong, frame color)
  /camera/depth/image_rect_raw (32FC1, frame depth)  /camera/depth/camera_info
  /camera/camera_info  (= color, untuk kompatibilitas)
  /scan, /wheel/odom, /imu/data (gyro+accel, orientation_covariance[0] = -1)
  /ground_truth/pose

---- dokumentasi asli ----
RTAB-Map RGB-D + EKF Driver Node (Layer 0 -> Layer 1 Raw & Layer 2 Optical)
Arsitektur: Pure Intel RealSense D455 (Registered RGB-D + CameraInfo + Internal IMU) + 3 Wheel Encoders.

Perbedaan Kunci dibandingkan Driver 2D Graph-SLAM:
1. TIDAK mem-broadcast TF (odom -> base_link) secara langsung, melainkan mem-publish
   /wheel/odom dan /imu/data agar dilebur terlebih dahulu oleh Node EKF (ekf_sensor_fusion.py).
2. Menyiarkan Static TF: base_link -> camera_link -> camera_optical_frame
   (Standar REP-103: +Z ke depan lensa, +X ke kanan gambar, +Y ke bawah gambar).
3. Mem-publish pasangan ter-registrasi (exact timestamp sync):
   - /camera/color/image_raw (bgr8, 640x480, FOV 86°)
   - /camera/depth/image_raw (32FC1 dalam meter, 640x480, FOV 86°)
   - /camera/camera_info     (Matriks kalibrasi intrinsik K dan P kamera Pinhole D455)
   - /scan                   (Irisan horizontal 2D untuk Hybrid ICP Reg/Strategy=2 di RTAB-Map)
"""

import os
import sys
import math
import ctypes
import numpy as np

if 'WEBOTS_HOME' not in os.environ:
    os.environ['WEBOTS_HOME'] = '/usr/local/webots'
sys.path.append(os.path.join(os.environ['WEBOTS_HOME'], 'lib', 'controller', 'python'))

from controller import Robot
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image, CameraInfo, LaserScan
import tf2_ros
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time as TimeMsg

# Pinjam modul matematika murni Layer 1 & Layer 2 dari folder utama (tanpa mengubahnya)
ROOT_SRC = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'src')
sys.path.append(ROOT_SRC)
from layer1_odometry import OmniImuOdometry
from layer2_d455_scan import D455DepthToScanConverter

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import d455_real_model as d455


def euler_to_quaternion(yaw: float, pitch: float = 0.0, roll: float = 0.0) -> Quaternion:
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class D455RtabmapDriverNode(Node):
    def __init__(self, robot: Robot):
        super().__init__('d455_rtabmap_driver')
        self.robot = robot
        self.timestep = int(self.robot.getBasicTimeStep())
        self.dt = self.timestep / 1000.0
        self.MAX_WHEEL_SPEED = 10.0

        # 1. Inisialisasi Hardware Layer 0 (Webots)
        self.motors = []
        self.encoders = []
        for i in range(1, 4):
            motor = self.robot.getDevice(f'wheel{i}')
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
            self.motors.append(motor)

            enc = self.robot.getDevice(f'wheel{i}_sensor')
            enc.enable(self.timestep)
            self.encoders.append(enc)

        self.imu = self.robot.getDevice('imu')
        self.imu.enable(self.timestep)
        self.gyro = self.robot.getDevice('gyro')
        self.gyro.enable(self.timestep)
        self.accel = self.robot.getDevice('accelerometer')
        self.accel.enable(self.timestep)

        # Profil laju data D455 (usb3 = datasheet, usb2 = hasil uji)
        self.profile_name = d455.get_profile_name()
        self.profile = d455.PROFILES[self.profile_name]
        self.realism = d455.realism_enabled()
        self.brake_dist = float(os.environ.get('D455_BRAKE_DIST', '0.40'))
        rng = np.random.default_rng(1234)
        self.cam_gate = d455.RateGate(self.profile['cam_hz'], self.profile['cam_drop'] if self.realism else 0.0, rng)
        self.gyro_gate = d455.RateGate(self.profile['gyro_hz'], 0.0, rng)
        self.accel_gate = d455.RateGate(self.profile['accel_hz'], 0.0, rng)

        cam_period_ms = max(self.timestep, int(round(1000.0 / self.profile['cam_hz'] / self.timestep)) * self.timestep)
        self.d455_color = self.robot.getDevice('d455_color')
        self.d455_color.enable(cam_period_ms)
        self.d455_depth = self.robot.getDevice('d455_depth')
        self.d455_depth.enable(cam_period_ms)

        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)

        # 2. Intrinsik (diturunkan dari FOV Webots = fx kamera asli)
        self.cam_w = self.d455_depth.getWidth()     # 640 px
        self.cam_h = self.d455_depth.getHeight()    # 480 px
        self.cam_fov = self.d455_depth.getFov()     # 1.38166 rad (79.16 deg)
        self.fx_d = (self.cam_w / 2.0) / math.tan(self.cam_fov / 2.0)          # ~387.07
        self.fx = (self.cam_w / 2.0) / math.tan(self.d455_color.getFov() / 2.0)  # ~379.91 (color)
        self.fy = self.fx
        self.cx = (self.cam_w - 1) / 2.0
        self.cy = (self.cam_h - 1) / 2.0
        self.depth_model = d455.D455DepthModel(self.cam_w, self.cam_h, self.fx_d, self.fx,
                                               enabled=self.realism)
        self.imu_model = d455.D455ImuModel(enabled=self.realism)
        self.gyro_yaw = 0.0          # yaw hasil integrasi gyro (bukan InertialUnit)
        self.last_gyro = np.zeros(3)
        self.last_accel = np.array([0.0, 0.0, 9.81])
        self.sim_time = 0.0

        self.odometry_model = OmniImuOdometry(wheel_radius=0.05, body_radius=0.12)
        self.scan_converter = D455DepthToScanConverter(
            width=self.cam_w,
            height=self.cam_h,
            hfov_rad=self.cam_fov,
            range_min=d455.DEPTH_MIN_M,
            range_max=self.d455_depth.getMaxRange(),
            num_beams=160
        )

        self.front_min_dist = float('inf')
        self.is_scene_stationary = False
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0
        self.init_gt_xy = None
        self.init_gt_yaw = None

        # 3. Publishers & Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Perhatikan: Driver mem-publish /wheel/odom (input mentah untuk EKF)
        self.wheel_odom_pub = self.create_publisher(Odometry, '/wheel/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        # Topik Kamera RGB-D Ter-registrasi + CameraInfo untuk Visual Odometry & RTAB-Map
        self.rgb_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/aligned_depth_to_color/image_raw', 10)
        self.depth_raw_pub = self.create_publisher(Image, '/camera/depth/image_rect_raw', 10)
        self.cam_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.color_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)
        self.gt_pub = self.create_publisher(PoseStamped, '/ground_truth/pose', 10)
        # Waktu simulasi -> /clock (node lain pakai use_sim_time:=true). Stamp semua
        # pesan = waktu Webots, supaya integrasi gyro/EKF tidak salah saat sim < real-time.
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)

        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._publish_static_tf_tree()

        self.step_count = 0
        self.get_logger().info('=============================================================')
        self.get_logger().info('  D455 (SPEK ASLI) RTAB-MAP + EKF DRIVER READY               ')
        self.get_logger().info(f'  - Profil    : {self.profile_name}  {self.profile}')
        self.get_logger().info(f'  - Realisme  : {"ON" if self.realism else "OFF (citra ideal)"}  | rem anti-tabrak {self.brake_dist:.2f} m')
        self.get_logger().info(f'  - Depth     : fx={self.fx_d:.2f} FOV={math.degrees(self.cam_fov):.2f} deg, min {d455.DEPTH_MIN_M} m')
        self.get_logger().info(f'  - Color     : fx={self.fx:.2f} FOV={math.degrees(self.d455_color.getFov()):.2f} deg, cx={self.cx:.1f}, cy={self.cy:.1f}')
        self.get_logger().info('=============================================================')

    def _publish_static_tf_tree(self):
        stamp = TimeMsg(sec=0, nanosec=0)   # static TF

        # 1. base_link -> camera_link (Bodi kamera D455: +X depan, +Y kiri, +Z atas)
        t_cam = TransformStamped()
        t_cam.header.stamp = stamp
        t_cam.header.frame_id = 'base_link'
        t_cam.child_frame_id = 'camera_link'
        t_cam.transform.translation.x = 0.08
        t_cam.transform.translation.y = 0.0
        t_cam.transform.translation.z = 0.10
        t_cam.transform.rotation.w = 1.0

        # 2. camera_link -> frame optik (REP-103: +Z depan, +X kanan, +Y bawah)
        #    camera_link = titik optik imager depth. Posisi color & IMU = -t ekstrinsik asli.
        def optical(child, parent, xyz):
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
            t.transform.rotation.x = -0.5
            t.transform.rotation.y = 0.5
            t.transform.rotation.z = -0.5
            t.transform.rotation.w = 0.5
            return t

        def link(child, parent, xyz):
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
            t.transform.rotation.w = 1.0
            return t

        tc = d455.T_DEPTH_TO_COLOR
        ti = d455.T_DEPTH_TO_IMU
        color_xyz = (float(tc[2]) * -1.0, float(tc[0]), float(tc[1]))   # (-z, x, y)_opt dari -t
        imu_xyz = (float(ti[2]) * -1.0, float(ti[0]), float(ti[1]))
        tfs = [
            t_cam,
            optical('camera_depth_optical_frame', 'camera_link', (0.0, 0.0, 0.0)),
            optical('camera_optical_frame', 'camera_link', (0.0, 0.0, 0.0)),  # nama lama
            link('camera_color_frame', 'camera_link', color_xyz),
            optical('camera_color_optical_frame', 'camera_color_frame', (0.0, 0.0, 0.0)),
            link('camera_imu_frame', 'camera_link', imu_xyz),
        ]
        self.static_tf_broadcaster.sendTransform(tfs)

    def _apply_wheel_commands(self):
        vx, vy, wz = self.cmd_vx, self.cmd_vy, self.cmd_wz
        if self.front_min_dist < self.brake_dist and vx > 0.0:
            vx = 0.0
        w1, w2, w3 = self.odometry_model.inverse_kinematics(vx, vy, wz)
        for motor, speed in zip(self.motors, [w1, w2, w3]):
            clamped = max(-self.MAX_WHEEL_SPEED, min(self.MAX_WHEEL_SPEED, speed))
            motor.setVelocity(clamped)

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_wz = msg.angular.z
        self._apply_wheel_commands()

    def sim_stamp(self) -> TimeMsg:
        t = self.robot.getTime()
        sec = int(t)
        return TimeMsg(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1000000000)

    def step(self):
        self.step_count += 1
        stamp = self.sim_stamp()
        self.clock_pub.publish(Clock(clock=stamp))

        # ---------------------------------------------------------------------
        # 1. LAYER 2: RGB-D seperti D455 asli + /scan (laju sesuai profil)
        # ---------------------------------------------------------------------
        self.sim_time = self.robot.getTime()
        if self.cam_gate.ready(self.sim_time):
            range_ptr = self.d455_depth.range_image
            color_bytes = self.d455_color.getImage()

            if range_ptr and color_bytes:
                w, h = self.cam_w, self.cam_h
                z_true = np.frombuffer(ctypes.string_at(range_ptr, w * h * 4), dtype=np.float32).reshape(h, w)

                # Depth "asli": noise, min 0.293 m, bayangan stereo, dropout (0 = kosong)
                z_depth = self.depth_model.degrade(z_true)
                z_aligned = self.depth_model.align_to_color(z_depth)

                # A. /scan & rem dari depth mentah (frame imager depth = camera_link)
                #    piksel kosong (0) -> NaN agar diabaikan seperti di kamera asli
                z_scan = z_depth.copy()
                z_scan[z_scan <= 0.0] = np.nan
                ranges, self.front_min_dist, self.is_scene_stationary = (
                    self.scan_converter.analyze_depth_and_extract_scan(z_scan.tobytes())
                )
                self._apply_wheel_commands()

                scan_msg = LaserScan()
                scan_msg.header.stamp = stamp
                scan_msg.header.frame_id = 'camera_link'
                scan_msg.angle_min = self.scan_converter.angle_min
                scan_msg.angle_max = self.scan_converter.angle_max
                scan_msg.angle_increment = self.scan_converter.angle_increment
                scan_msg.time_increment = 0.0
                scan_msg.scan_time = self.cam_gate.period
                scan_msg.range_min = self.scan_converter.range_min
                scan_msg.range_max = self.scan_converter.range_max
                scan_msg.ranges = ranges
                self.scan_pub.publish(scan_msg)

                # B. Depth ter-align ke color (dipakai VO & RTAB-Map) + depth mentah
                self.depth_pub.publish(self._depth_msg(z_aligned, stamp, 'camera_color_optical_frame'))
                self.depth_raw_pub.publish(self._depth_msg(z_depth, stamp, 'camera_depth_optical_frame'))

                # C. Konversi Citra Warna Webots (BGRA8) -> BGR8
                bgra = np.frombuffer(color_bytes, dtype=np.uint8).reshape((h, w, 4))
                bgr = np.ascontiguousarray(bgra[:, :, :3])

                rgb_msg = Image()
                rgb_msg.header.stamp = stamp
                rgb_msg.header.frame_id = 'camera_color_optical_frame'
                rgb_msg.height = h
                rgb_msg.width = w
                rgb_msg.encoding = 'bgr8'
                rgb_msg.is_bigendian = False
                rgb_msg.step = w * 3
                rgb_msg.data = bgr.tobytes()
                self.rgb_pub.publish(rgb_msg)

                # D. CameraInfo (color untuk RGB-D ter-align, depth untuk citra mentah)
                ci_color = self._camera_info(stamp, 'camera_color_optical_frame', self.fx)
                self.color_info_pub.publish(ci_color)
                self.cam_info_pub.publish(ci_color)
                self.depth_info_pub.publish(self._camera_info(stamp, 'camera_depth_optical_frame', self.fx_d))

        # ---------------------------------------------------------------------
        # 2. LAYER 1 RAW: Odometri Roda (/wheel/odom) & IMU (/imu/data) untuk EKF
        # ---------------------------------------------------------------------
        phi1 = self.encoders[0].getValue()
        phi2 = self.encoders[1].getValue()
        phi3 = self.encoders[2].getValue()
        rpy = self.imu.getRollPitchYaw()   # HANYA ground truth
        gyro_true = self.gyro.getValues()
        accel_true = self.accel.getValues()

        if any(math.isnan(v) for v in [phi1, phi2, phi3]) or any(math.isnan(v) for v in gyro_true):
            return

        # Gyro D455 (bias + noise). Sampel ditahan (zero-order hold) di antara update
        # sesuai laju profil; yaw diintegrasi dari gyro -> drift seperti kamera asli.
        gyro_new = self.gyro_gate.ready(self.sim_time)
        if gyro_new:
            self.last_gyro = self.imu_model.gyro(gyro_true)
        if self.accel_gate.ready(self.sim_time):
            self.last_accel = self.imu_model.accel(accel_true)
        self.gyro_yaw = math.atan2(math.sin(self.gyro_yaw + self.last_gyro[2] * self.dt),
                                   math.cos(self.gyro_yaw + self.last_gyro[2] * self.dt))

        x, y, theta, vx, vy, wz = self.odometry_model.update(
            phi1, phi2, phi3, self.gyro_yaw, self.dt, is_stalled=self.is_scene_stationary
        )
        q_odom = euler_to_quaternion(theta)

        wheel_odom = Odometry()
        wheel_odom.header.stamp = stamp
        wheel_odom.header.frame_id = 'odom'
        wheel_odom.child_frame_id = 'base_link'
        wheel_odom.pose.pose.position.x = x
        wheel_odom.pose.pose.position.y = y
        wheel_odom.pose.pose.orientation = q_odom
        wheel_odom.pose.covariance = self.odometry_model.get_pose_covariance_6x6()
        wheel_odom.twist.twist.linear.x = vx
        wheel_odom.twist.twist.linear.y = vy
        wheel_odom.twist.twist.angular.z = wz
        self.wheel_odom_pub.publish(wheel_odom)

        if gyro_new:
            imu_msg = Imu()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = 'camera_imu_frame'
            # D455 tidak punya orientasi -> REP-145: orientation_covariance[0] = -1
            imu_msg.orientation.w = 1.0
            imu_msg.orientation_covariance[0] = -1.0
            g_std = d455.GYRO_NOISE_STD ** 2
            a_std = d455.ACCEL_NOISE_STD ** 2
            imu_msg.angular_velocity_covariance[0] = float(g_std[0])
            imu_msg.angular_velocity_covariance[4] = float(g_std[1])
            imu_msg.angular_velocity_covariance[8] = float(g_std[2])
            imu_msg.linear_acceleration_covariance[0] = float(a_std[0])
            imu_msg.linear_acceleration_covariance[4] = float(a_std[1])
            imu_msg.linear_acceleration_covariance[8] = float(a_std[2])
            imu_msg.angular_velocity.x = float(self.last_gyro[0])
            imu_msg.angular_velocity.y = float(self.last_gyro[1])
            imu_msg.angular_velocity.z = float(self.last_gyro[2])
            imu_msg.linear_acceleration.x = float(self.last_accel[0])
            imu_msg.linear_acceleration.y = float(self.last_accel[1])
            imu_msg.linear_acceleration.z = float(self.last_accel[2])
            self.imu_pub.publish(imu_msg)

        # 3. Ground Truth Pose (Referensi Evaluasi di RViz2)
        gt_vals = self.gps.getValues()
        if gt_vals and not math.isnan(gt_vals[0]):
            if self.init_gt_xy is None:
                self.init_gt_xy = (gt_vals[0], gt_vals[1])
                self.init_gt_yaw = rpy[2]
            dx_w = gt_vals[0] - self.init_gt_xy[0]
            dy_w = gt_vals[1] - self.init_gt_xy[1]
            c0, s0 = math.cos(self.init_gt_yaw), math.sin(self.init_gt_yaw)
            gt_x = dx_w * c0 + dy_w * s0
            gt_y = -dx_w * s0 + dy_w * c0

            gt_msg = PoseStamped()
            gt_msg.header.stamp = stamp
            gt_msg.header.frame_id = 'map'
            gt_msg.pose.position.x = gt_x
            gt_msg.pose.position.y = gt_y
            gt_msg.pose.position.z = 0.0
            gt_msg.pose.orientation = euler_to_quaternion(rpy[2] - self.init_gt_yaw)
            self.gt_pub.publish(gt_msg)

    def _depth_msg(self, z: np.ndarray, stamp, frame: str) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame
        msg.height, msg.width = z.shape
        msg.encoding = '32FC1'
        msg.is_bigendian = False
        msg.step = z.shape[1] * 4
        msg.data = np.ascontiguousarray(z, dtype=np.float32).tobytes()
        return msg

    def _camera_info(self, stamp, frame: str, f: float) -> CameraInfo:
        ci = CameraInfo()
        ci.header.stamp = stamp
        ci.header.frame_id = frame
        ci.height = self.cam_h
        ci.width = self.cam_w
        ci.distortion_model = 'plumb_bob'
        ci.d = [0.0, 0.0, 0.0, 0.0, 0.0]   # citra Webots tidak berdistorsi
        ci.k = [f, 0.0, self.cx, 0.0, f, self.cy, 0.0, 0.0, 1.0]
        ci.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        ci.p = [f, 0.0, self.cx, 0.0, 0.0, f, self.cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return ci


def main(args=None):
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    rclpy.init(args=args)
    node = D455RtabmapDriverNode(robot)

    try:
        while rclpy.ok() and robot.step(timestep) != -1:
            rclpy.spin_once(node, timeout_sec=0.0)
            node.step()
    except KeyboardInterrupt:
        pass
    finally:
        for m in node.motors:
            m.setVelocity(0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
